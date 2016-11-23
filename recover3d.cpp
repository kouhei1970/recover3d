//三次元復元研究用プログラム
//２枚の透視投影画像から回転量と移動量を求める関数（２０１６．１１．２３）
#include "recover3d.h"

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

//座標の設定はカメラの前方方向がｚ軸
//前方に向いて右向きがx軸
//前方に向いて上向きがｙ軸
//戻り値のRretは１次元配列のポインタ．Rは３×３の行列，行を横に並べて１次元配列にして返す
//戻り値のTretは１次元配列のポインタ．Tは３次元ベクトル，要素３個の配列で返す．ｘ，ｙ，ｚの順
//戻り値P3Dは１次元配列のポインタ．x0,y0,z0,x1,y1,z1,x2,y2,z2,.....というふうに推定した３次元点がNの数だけ並んでいる．
//u1,v1,u2,v2は１次元配列．要素数は特徴点の数に一致し引数Nと同じでなければならない
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
)
{

  //内部パラメータ行列作成
  float fx=focal;//300.0;
  float fy=focal;//300.0;
  float cx=centerx;//0.0;
  float cy=centery;//0.0;
  Mat K = (Mat_<float>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

  //外部パラメータ行列１作成
  Mat rvec1=(Mat_<float>(3,1)<<0,0,0);//カメラ１の姿勢（すべて０）
  Mat tvec1=(Mat_<float>(3,1)<<0,0,0);//カメラ１の位置（すべて０）


  //引数からopenCVの関数に渡すための準備
  vector<Point2f> points1;
  vector<Point2f> points2;
  for (int i=0;i<N;i++){
    points1.push_back(Point2f(u1[i],v1[i]));
    points2.push_back(Point2f(u2[i],v2[i]));
  }

  //5点法及びランザックアルゴリズムを用いてエッセンシャル行列,Rとt を求める
  Mat R,t,mask,points3d,points4d;
  Mat E=findEssentialMat(points1,points2,focal,Point2d(cx,cy),RANSAC,0.999,1.0,mask);
  recoverPose(E,points1,points2,R,t,focal,Point2d(cx,cy),mask);

  //投影行列の作成
  Mat M1(3,4,CV_32F);
  Mat M2(3,4,CV_32F);
  Mat R1(3,3,CV_32F);
  //Mat R2(3,3,CV_32F);
  Mat T1(3,1,CV_32F);
  Mat T2(3,1,CV_32F);

  Rodrigues(rvec1,R1);  
  t=scale*t;

  for (int m=0; m<3;m++){
    for (int n=0; n<4; n++){
      if(n==3){
        M1.at<float>(m,n)=tvec1.at<float>(m,0);
        M2.at<float>(m,n)=t.at<double>(m,0);
      }
      else {
        M1.at<float>(m,n)=R1.at<float>(m,n);
        M2.at<float>(m,n)=R.at<double>(m,n);
      }
    }
  }
  
  M1=K*M1;
  M2=K*M2;
  
  //三角法で３次元復元  
  triangulatePoints(M1, M2, points1, points2, points4d);
  convertPointsFromHomogeneous(points4d.t(),points3d);

  for (int m=0; m<3; m++) {
    Tret[m]=t.at<double>(m,0);
    
    for(int n=0; n<3; n++) {
      Rret[m*3+n]=R.at<double>(m,n);
    }
  }

  for (int i = 0; i<N; i++) {

      P3D[ i * 3 + 0] = points3d.at<float>(i,0);
      P3D[ i * 3 + 1] = points3d.at<float>(i,1);
      P3D[ i * 3 + 2] = points3d.at<float>(i,2);

  }
  
  return 0;

}
