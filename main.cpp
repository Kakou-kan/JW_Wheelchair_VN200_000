/*
 * 山田さんの作ったアカデミックパックのプログラムのラッパーのテストプログラム
 * Author: suzuryo Date: 2015.5.26
 * 
 * 必要ライブラリ: jwvechile_serial.cpp jwvehicle_serial_protocol.cpp
 * 必要ヘッダ: jwvechile_serial.hpp jwvehicle_serial_protocol.hpp
 */

#include<Windows.h>
#include<process.h>

#include<stdio.h>

#include"jwvehicle_wrapper.h"

#include<iostream>
#include "vectornav.h"
#include<cmath>
#include <iomanip> 
using namespace std;

const char* const COM_PORT = "\\\\.\\COM3";
const int BAUD_RATE = 115200;
const double EARTH_R = 6378.137;
double cal_distance(double lat1, double lon1, double lat2, double lon2);//latは纬度、lonは经度
double cal_azimuth(double lat1, double lon1, double lat2, double lon2);

int main(){
	JWVehicle vehicle("\\\\.\\COM9");
	
	//まずは止める
	vehicle.move(0,0);
	Sleep(100);

	//5秒右回り
	vehicle.move(0,-100);
	Sleep(5000);

	//3秒半分スピードで前進
	vehicle.move(50,0);
	Sleep(5000);

	//止める
	vehicle.move(0,0);
	Sleep(100);

	//GPS信号確認
	Vn200 vn200;
	VnDeviceCompositeData data;

	vn200_connect(&vn200, COM_PORT, BAUD_RATE);
	vn200_setAsynchronousDataOutputType(
		&vn200,
		//VNASYNC_VNYMR,
		VNASYNC_VNGPS,
		true);
	Sleep(1000);
	VN_ERROR_CODE err = vn200_getCurrentAsyncData(&vn200, &data);
	if (err != VNERR_NO_ERROR) {
		printf("Error on vn200_getCurrentAsyncData: %d\n", err);
	}
	else {

		printf("Orientation YawPitchRoll: %+#7.2f %+#7.2f %+#7.2f\n", data.ypr.yaw, data.ypr.pitch, data.ypr.roll);
		printf("Acceleration XYZ: %+#7.2f %+#7.2f %+#7.2f\n", data.acceleration.c0, data.acceleration.c1, data.acceleration.c2);
		printf("Location: %+#7.2f %+#7.2f %+#7.2f\n", data.latitudeLongitudeAltitude.c0, data.latitudeLongitudeAltitude.c1, data.latitudeLongitudeAltitude.c2);
	}

	//起点GPS座標を記録する
	double stgpsc0 = data.latitudeLongitudeAltitude.c0;
	double stgpsc1 = data.latitudeLongitudeAltitude.c1;
	double stgpsc2 = data.latitudeLongitudeAltitude.c2;

	//終点GPS座標を記録する
	double edgpsc0 = 0.0;
	double edgpsc1 = 0.0;
	double edgpsc2 = 0.0;
	printf("latitude: \n");
	cin>> edgpsc0;
	printf("longitude: \n");
	cin >> edgpsc1;
	printf("altitude：\n");
	cin >> edgpsc2;

	//試運転
	vehicle.move(50, 0);
	Sleep(5000);

	//中間点GPS座標を記録する
	double midgpsc0 = data.latitudeLongitudeAltitude.c0;
	double midgpsc1 = data.latitudeLongitudeAltitude.c1;
	double midgpsc2 = data.latitudeLongitudeAltitude.c2;

	//距離を計算する
	double dist_strt_end = cal_distance(stgpsc0, stgpsc1, edgpsc0, edgpsc1);
	double dist_strt_mid = cal_distance(stgpsc0, stgpsc1, midgpsc0, midgpsc1);
	double dist_mid_end = cal_distance(midgpsc0, midgpsc1, edgpsc0, edgpsc1);

	//方位角を計算する
	double azi_strt_end = cal_azimuth(stgpsc0, stgpsc1, edgpsc0, edgpsc1);
	double azi_strt_mid = cal_azimuth(stgpsc0, stgpsc1, midgpsc0, midgpsc1);
	double azi_mid_end = cal_azimuth(midgpsc0, midgpsc1, edgpsc0, edgpsc1);
	
	//回転角度を計算する
	double strgangle = azi_mid_end - azi_strt_mid;

	//回転
	vehicle.move(0, -100);
	Sleep(5000); // i don't know the specific angle of "-100"

	//go
	vehicle.move(100, 0);
	Sleep(dist_mid_end / 100);
	vehicle.move(0, 0);
	Sleep(100);

	//誤差検査


	vn200_disconnect(&vn200);
	return 0;

}

double cal_distance(double lat1, double lon1, double lat2, double lon2) {
	double latitude1;//纬度
	double longitude1;//经度
	double latitude2;//纬度
	double longitude2;//经度
	latitude1 = lat1 * 3.14 / 180;
	latitude2 = lat2 * 3.14 / 180;
	longitude1 = lon1 * 3.14 / 180;
	longitude2 = lon2 * 3.14 / 180;
	double cosd = cos(latitude2) * cos(latitude1) * cos(longitude2 - longitude1) + sin(latitude1) * sin(latitude2);
	double distance = EARTH_R * acos(cosd);
	return distance;
}

double cal_azimuth(double lat1, double lon1, double lat2, double lon2) {
	double latitude1;//纬度
	double longitude1;//经度
	double latitude2;//纬度
	double longitude2;//经度
	latitude1 = lat1 * 3.14 / 180;
	latitude2 = lat2 * 3.14 / 180;
	longitude1 = lon1 * 3.14 / 180;
	longitude2 = lon2 * 3.14 / 180;
	double Azimuth = 0.0;
	double lat_c = (latitude1 + latitude2) / 2;
	double dx = EARTH_R * (longitude2 - longitude1) * cos(lat_c);
	double dy = EARTH_R * (latitude2 - latitude1);
	Azimuth = 90 - atan2(dy, dx) / 3.14 * 180;
	return Azimuth ;
}
