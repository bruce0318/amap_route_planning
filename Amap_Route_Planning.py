# -*- coding: utf-8 -*-
"""
/***********************************************
 AmapRoutePlanning
    QGIS插件：高德地图API路径规划
    支持驾车/步行/骑行三种方式，批量OD分析，导出CSV，路径图层绘制
    作者：MaritimeDay
    邮箱：Connor_rk800A@163.com
 ***********************************************/
"""
from qgis.PyQt.QtCore import QSettings, QTranslator, QCoreApplication, QThread, pyqtSignal
from qgis.PyQt.QtGui import QIcon
from qgis.PyQt.QtWidgets import QAction, QFileDialog, QMessageBox
from qgis.core import (
    QgsProject, QgsVectorLayer, QgsFeature, QgsGeometry, QgsPointXY,
    QgsFields, QgsField, QgsWkbTypes, QgsCoordinateReferenceSystem, QgsCoordinateTransform
)
from .resources import *
from .Amap_Route_Planning_dialog import RoutePlanningDialog
import os.path
import requests
import csv
import time
from .transform import gcj2wgs

class RouteWorker(QThread):
    progress = pyqtSignal(int, int)
    finished = pyqtSignal(list)
    error = pyqtSignal(str)
    log = pyqtSignal(str)
    set_progress = pyqtSignal(int)

    def __init__(self, od_pairs, mode, crs_src, amap_key, origin_field, dest_field, city=None):
        super().__init__()
        self.od_pairs = od_pairs
        self.mode = mode
        self.crs_src = crs_src
        self.amap_key = amap_key
        self.origin_field = origin_field
        self.dest_field = dest_field
        self.city = city
        self.stop_flag = False

    def wgs84_to_gcj02(self, lon, lat):
        url = f'https://restapi.amap.com/v3/assistant/coordinate/convert?locations={lon},{lat}&coordsys=gps&key={self.amap_key}'
        resp = requests.get(url).json()
        if resp['status'] == '1':
            lon_gcj, lat_gcj = map(float, resp['locations'].split(','))
            return lon_gcj, lat_gcj
        else:
            raise Exception('坐标转换失败: ' + resp.get('info', ''))

    def get_route_amap(self, origin, destination, mode='driving', city=None):
        if mode == 'bicycling':
            url = f'https://restapi.amap.com/v4/direction/bicycling?origin={origin[0]},{origin[1]}&destination={destination[0]},{destination[1]}&key={self.amap_key}'
            resp = requests.get(url).json()
            if resp.get('errcode', 1) == 0 and resp['data']['paths']:
                route = resp['data']['paths'][0]
                distance = float(route['distance'])
                duration = float(route['duration'])
                points = []
                steps = []
                for idx, step in enumerate(route['steps']):
                    steps.append({
                        'step_no': idx+1,
                        'distance': step.get('distance'),
                        'duration': step.get('duration'),
                        'instruction': step.get('instruction'),
                        'road_name': step.get('road', ''),
                        'polyline': step.get('polyline', '')
                    })
                    for pt in step['polyline'].split(';'):
                        lon, lat = map(float, pt.split(','))
                        points.append((lon, lat))
                return distance, duration, points, steps
            else:
                raise Exception('路径规划失败: ' + str(resp.get('errmsg', resp.get('info', '未知错误'))))
        elif mode == 'transit':
            if not city:
                raise Exception('公交模式下城市不能为空')
            url = f'https://restapi.amap.com/v3/direction/transit/integrated?origin={origin[0]},{origin[1]}&destination={destination[0]},{destination[1]}&city={city}&key={self.amap_key}'
            resp = requests.get(url).json()
            if resp['status'] == '1' and resp['route']['transits']:
                transit = resp['route']['transits'][0]
                distance = float(transit['distance'])
                duration = float(transit['duration'])
                steps = []
                polyline = []
                for idx, seg in enumerate(transit['segments']):
                    instr = ''
                    pl = ''
                    if 'bus' in seg and seg['bus']['buslines']:
                        instr = seg['bus']['buslines'][0]['name']
                        pl = seg['bus']['buslines'][0]['polyline']
                    elif 'walking' in seg and seg['walking']['steps']:
                        instr = '步行'
                        pl = ';'.join([step['polyline'] for step in seg['walking']['steps']])
                    steps.append({
                        'step_no': idx+1,
                        'instruction': instr,
                        'polyline': pl
                    })
                    for pt in pl.split(';'):
                        if pt:
                            lon, lat = map(float, pt.split(','))
                            polyline.append((lon, lat))
                return distance, duration, polyline, steps
            else:
                raise Exception('公交路径规划失败: ' + resp.get('info', ''))
        else:
            url = f'https://restapi.amap.com/v3/direction/{mode}?origin={origin[0]},{origin[1]}&destination={destination[0]},{destination[1]}&key={self.amap_key}'
            resp = requests.get(url).json()
            if resp['status'] == '1' and resp['route']['paths']:
                route = resp['route']['paths'][0]
                distance = float(route['distance'])
                duration = float(route['duration'])
                points = []
                steps = []
                for idx, step in enumerate(route['steps']):
                    steps.append({
                        'step_no': idx+1,
                        'distance': step.get('distance'),
                        'duration': step.get('duration'),
                        'instruction': step.get('instruction'),
                        'road_name': step.get('road', ''),
                        'polyline': step.get('polyline', '')
                    })
                    for pt in step['polyline'].split(';'):
                        lon, lat = map(float, pt.split(','))
                        points.append((lon, lat))
                return distance, duration, points, steps
            else:
                raise Exception('路径规划失败: ' + resp.get('info', ''))

    def run(self):
        from qgis.core import QgsCoordinateReferenceSystem, QgsCoordinateTransform, QgsPointXY
        results = []
        crs_wgs = QgsCoordinateReferenceSystem('EPSG:4326')
        xform_to_wgs = QgsCoordinateTransform(self.crs_src, crs_wgs, QgsProject.instance())
        xform_to_src = QgsCoordinateTransform(crs_wgs, self.crs_src, QgsProject.instance())
        total = len(self.od_pairs)
        for idx, (f1, f2) in enumerate(self.od_pairs):
            if getattr(self, 'stop_flag', False):
                self.log.emit('用户中断，提前结束')
                break
            try:
                o_pt = f1.geometry().asPoint()
                d_pt = f2.geometry().asPoint()
                o_wgs = xform_to_wgs.transform(QgsPointXY(o_pt))
                d_wgs = xform_to_wgs.transform(QgsPointXY(d_pt))
                o_gcj = self.wgs84_to_gcj02(o_wgs.x(), o_wgs.y())
                d_gcj = self.wgs84_to_gcj02(d_wgs.x(), d_wgs.y())
                if self.mode == 'transit':
                    distance, duration, polyline, steps = self.get_route_amap(o_gcj, d_gcj, self.mode, self.city)
                else:
                    distance, duration, polyline, steps = self.get_route_amap(o_gcj, d_gcj, self.mode)
                # polyline GCJ-02 -> WGS84 -> 工程坐标系
                line_points = []
                for lon_gcj, lat_gcj in polyline:
                    lon_wgs, lat_wgs = gcj2wgs(lon_gcj, lat_gcj)
                    pt_wgs = QgsPointXY(lon_wgs, lat_wgs)
                    pt_proj = xform_to_src.transform(pt_wgs)
                    line_points.append(pt_proj)
                origin_attr = f1[self.origin_field]
                dest_attr = f2[self.dest_field]
                results.append({
                    'origin_attr': origin_attr,
                    'dest_attr': dest_attr,
                    'distance': distance,
                    'duration': duration,
                    'geometry': line_points,
                    'steps': steps
                })
                self.log.emit(f'OD({origin_attr})→({dest_attr}) 距离:{distance}m 用时:{duration}s 成功')
            except Exception as e:
                origin_attr = f1[self.origin_field]
                dest_attr = f2[self.dest_field]
                self.log.emit(f'OD({origin_attr}→{dest_attr}) 失败: {e}')
            self.progress.emit(idx+1, total)
            self.set_progress.emit(int((idx+1)/total*100))
            import time
            time.sleep(0.2)
        self.finished.emit(results)

class RoutePlanning:
    def __init__(self, iface):
        self.iface = iface
        self.plugin_dir = os.path.dirname(__file__)
        locale = QSettings().value('locale/userLocale')[0:2]
        locale_path = os.path.join(self.plugin_dir, 'i18n', f'AmapRoutePlanning_{locale}.qm')
        if os.path.exists(locale_path):
            self.translator = QTranslator()
            self.translator.load(locale_path)
            QCoreApplication.installTranslator(self.translator)
        self.actions = []
        self.menu = self.tr(u'&Amap Route Planning')
        self.first_start = None

    def tr(self, message):
        return QCoreApplication.translate('AmapRoutePlanning', message)

    def add_action(self, icon_path, text, callback, enabled_flag=True, add_to_menu=True, add_to_toolbar=True, status_tip=None, whats_this=None, parent=None):
        icon = QIcon(icon_path)
        action = QAction(icon, text, parent)
        action.triggered.connect(callback)
        action.setEnabled(enabled_flag)
        if status_tip is not None:
            action.setStatusTip(status_tip)
        if whats_this is not None:
            action.setWhatsThis(whats_this)
        if add_to_toolbar:
            self.iface.addToolBarIcon(action)
        if add_to_menu:
            self.iface.addPluginToMenu(self.menu, action)
        self.actions.append(action)
        return action

    def initGui(self):
        icon_path = ':/plugins/Route_Planning/icon.png'
        self.add_action(icon_path, text=self.tr(u'Route Planning'), callback=self.run, parent=self.iface.mainWindow())
        self.first_start = True

    def unload(self):
        for action in self.actions:
            self.iface.removePluginMenu(self.tr(u'&Route Planning'), action)
            self.iface.removeToolBarIcon(action)

    def run(self):
        if self.first_start:
            self.first_start = False
            self.dlg = RoutePlanningDialog()
            self.dlg.btn_run.clicked.connect(self.run_analysis)
            self.dlg.btn_stop.clicked.connect(self.stop_analysis)
            self.dlg.btn_browse.clicked.connect(self.browse_output)
        self.dlg.set_run_enabled(True)
        self.dlg.reset_progress()
        self.dlg.textEdit_log.clear()
        self.dlg.show()

    def browse_output(self):
        # 用self.dlg作为父窗口，避免窗口缩小
        path, _ = QFileDialog.getSaveFileName(self.dlg, '导出CSV', '', 'CSV Files (*.csv)')
        if path:
            self.dlg.lineEdit_output_path.setText(path)

    def stop_analysis(self):
        if hasattr(self, 'worker') and self.worker.isRunning():
            self.worker.stop_flag = True
            self.dlg.append_log('用户请求中断，正在停止...')

    def run_analysis(self):
        self.dlg.set_run_enabled(False)
        key = self.dlg.get_key()
        origin_layer_name = self.dlg.get_origin_layer()
        dest_layer_name = self.dlg.get_dest_layer()
        origin_field = self.dlg.get_origin_field()
        dest_field = self.dlg.get_dest_field()
        mode = self.dlg.get_mode()
        export_layer = self.dlg.get_export_layer()
        output_path = self.dlg.get_output_path()
        city = self.dlg.get_city() if hasattr(self.dlg, 'get_city') else None
        if not key or not origin_layer_name or not dest_layer_name or not origin_field or not dest_field:
            self.dlg.append_log('请完整选择参数')
            self.dlg.set_run_enabled(True)
            return
        from qgis.core import QgsProject
        origin_layer = None
        dest_layer = None
        for lyr in QgsProject.instance().mapLayers().values():
            if lyr.name() == origin_layer_name:
                origin_layer = lyr
            if lyr.name() == dest_layer_name:
                dest_layer = lyr
        if not origin_layer or not dest_layer:
            self.dlg.append_log('图层不存在')
            self.dlg.set_run_enabled(True)
            return
        origin_features = list(origin_layer.getFeatures())
        dest_features = list(dest_layer.getFeatures())
        od_pairs = []
        for f1 in origin_features:
            for f2 in dest_features:
                od_pairs.append((f1, f2))
        crs_src = origin_layer.crs()
        self.worker = RouteWorker(od_pairs, mode, crs_src, key, origin_field, dest_field, city)
        self.worker.progress.connect(self.on_progress)
        self.worker.finished.connect(lambda results: self.on_finished(results, export_layer, output_path, origin_field, dest_field))
        self.worker.error.connect(self.on_error)
        self.worker.log.connect(self.dlg.append_log)
        self.worker.set_progress.connect(self.dlg.set_progress)
        self.worker.stop_flag = False
        self.worker.start()

    def on_progress(self, done, total):
        percent = int(done / total * 100) if total else 0
        self.dlg.set_progress(percent)

    def on_error(self, msg):
        self.dlg.append_log(msg)
        self.dlg.set_run_enabled(True)

    def on_finished(self, results, export_layer, output_path, origin_field, dest_field):
        self.dlg.set_run_enabled(True)
        # 1. 路径图层（仅有有效路径时生成）
        if export_layer and any(r['geometry'] for r in results):
            import math
            crs_authid = self.worker.crs_src.authid()
            from qgis.core import QgsVectorLayer, QgsField, QgsFeature, QgsGeometry, QgsProject
            fields = [QgsField(f'origin_{origin_field}', 10), QgsField(f'dest_{dest_field}', 10), QgsField('distance', 6, 'double'), QgsField('duration', 6, 'double')]
            vl = QgsVectorLayer(f'LineString?crs={crs_authid}', 'AMAP路径规划', 'memory')
            pr = vl.dataProvider()
            pr.addAttributes(fields)
            vl.updateFields()
            feats = []
            for r in results:
                if r['geometry']:
                    # 过滤无效点
                    valid_points = [pt for pt in r['geometry'] if pt.x() is not None and pt.y() is not None and not math.isnan(pt.x()) and not math.isnan(pt.y())]
                    if not valid_points:
                        continue
                    feat = QgsFeature()
                    feat.setGeometry(QgsGeometry.fromPolylineXY(valid_points))
                    attrs = [r['origin_attr'], r['dest_attr'], r['distance'], r['duration']]
                    feat.setAttributes(attrs)
                    feats.append(feat)
            if feats:
                pr.addFeatures(feats)
                vl.updateExtents()
                QgsProject.instance().addMapLayer(vl)
                self.dlg.append_log(f'已生成路径图层，共{len(feats)}条')
        # 2. 导出OD汇总CSV
        if output_path:
            import csv
            with open(output_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                header = [f'origin_{origin_field}', f'dest_{dest_field}', 'distance', 'duration']
                writer.writerow(header)
                for r in results:
                    row = [r['origin_attr'], r['dest_attr'], r['distance'], r['duration']]
                    writer.writerow(row)
            self.dlg.append_log(f'已导出CSV：{output_path}')
        # 3. 导出step表格
        if output_path:
            steps_path = output_path.replace('.csv', '_steps.csv')
            with open(steps_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([f'origin_{origin_field}', f'dest_{dest_field}', 'step_no', 'distance', 'duration', 'instruction', 'road_name', 'polyline'])
                for r in results:
                    for step in r.get('steps', []):
                        writer.writerow([
                            r['origin_attr'], r['dest_attr'], step.get('step_no'), step.get('distance'), step.get('duration'),
                            step.get('instruction'), step.get('road_name'), step.get('polyline')
                        ])
            self.dlg.append_log(f'已导出step表格：{steps_path}')
        self.dlg.append_log('分析完成')
