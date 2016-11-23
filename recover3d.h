#ifndef RECOVER3D_H
#define RECOVER3D_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include "opencv2/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"

int recover3d(
              double *Rret,     //R行列の計算結果の戻り値を受け取る配列のポインタ
              double *Tret,     //T行列の計算結果の戻り値を受け取る配列のポインタ
              double *P3D,      //復元された３次元点群の値を受け取る配列のポインタ
              double scale,     //移動量
              double focal,     //焦点距離
              double centerx,   //画像中心ｘ
              double centery,   //画像中心ｙ
              int N,            //対応が付いている特徴点の数
              double *u1,       //１枚目の画像の特徴点ｘ座標配列のポインタ
              double *v1,       //１枚目の画像の特徴点ｙ座標配列のポインタ
              double *u2,       //２枚目の画像の特徴点ｘ座標配列のポインタ
              double *v2        //２枚目の画像の特徴点ｙ座標配列のポインタ
);

#endif

