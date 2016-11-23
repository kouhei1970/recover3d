//
//  関数recover3d用のテストプログラム
//  使用法サンプル
//
//  本プログラムの概要
//    ランダムに３次元点群を生成し
//    それらを違う位置で撮影した画像上の座標を計算し
//    それらの２つの位置において撮影した特徴点座標を関数recover3dに与えることで
//    R,Tと元の３次元点を推定し出力するプログラム

//関数recover3dを使用する際は以下のようにヘッダファイルを取り込んでください．
#include "recover3d.h"

//openCVを使用する際はオリジナルプログラムにも以下の３行を必ず冒頭に書いてください．
using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

//以下のclassは本テストプログラムのためだけに必要ですので，考慮しなくても良いです．
//ランダムな３次元点群を生成するためのクラス
class my3dpoint{
  double x,y,z;

public:

  my3dpoint();

  double getX(){return x;}
  double getY(){return y;}
  double getZ(){return z;}
  void disp(){cout<<"["<<x<<","<<y<<","<<z<<"]"<<std::endl;}
  void dispnobr(){cout<<x<<" "<<y<<" "<<z<<std::endl;}
};

my3dpoint::my3dpoint(){
  double harf=RAND_MAX/2.0;
  x=((rand()-harf)/harf)*200.0;
  y=((rand()-harf)/harf)*200.0;
  z=300.0+((double)rand()/RAND_MAX)*500.0;
}

//以下からテストプログラムのメインです．
//---------- main ----------
int main()
{
  int point_count = 10;//生成するランダムな３D点の数
  double u1[point_count], v1[point_count], u2[point_count], v2[point_count];

  my3dpoint p1[point_count];
  vector<Point2f> points1;
  vector<Point2f> points2;
  vector<Point3f> point3d1(point_count);
  Mat points4d;
  Mat points3d;

  //3D点ベクトル設定
  for(int i=0; i < point_count; i++){
    point3d1[i]=Point3f(p1[i].getX(),p1[i].getY(),p1[i].getZ());
  }

  //内部パラメータ行列作成
  float focal = 300.0;
  float fx=focal;
  float fy=focal;
  float cx=0.0;
  float cy=0.0;
  Mat K = (Mat_<float>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  
  //外部パラメータ行列１作成  
  Mat rvec1=(Mat_<float>(3,1)<<0,0,0);//第１カメラの移動量０
  Mat tvec1=(Mat_<float>(3,1)<<0,0,0);//第１カメラの回展０

  //外部パラメータ行列２作成
  float tx=10,ty=0,tz=10;//第２カメラの移動量
  float rx=0.0,ry=0.0*3.14159/180,rz=0.0*3.14159/180;//第２カメラの回転量
  float scale=sqrt(tx*tx+ty*ty+tz*tz);
  Mat rvec2=(Mat_<float>(3,1)<<rx,ry,rz);
  Mat tvec2=(Mat_<float>(3,1)<<tx,ty,tz);

  //２画面へ透視投影
  Mat distcoef;
  projectPoints(point3d1,rvec1,tvec1,K,distcoef,points1);
  projectPoints(point3d1,rvec2,tvec2,K,distcoef,points2);

  for (int i=0; i<point_count; i++){
    u1[i] = points1[i].x;
    v1[i] = points1[i].y;
    u2[i] = points2[i].x;
    v2[i] = points2[i].y;
  }

//２枚の画像の対応点からそれらの点の３次元復元を実施
  double Rret[9],Tret[3],P3D[point_count*3];
  recover3d(
              Rret,       //R行列の計算結果の戻り値を受け取る配列のポインタ
              Tret,       //T行列の計算結果の戻り値を受け取る配列のポインタ
              P3D,        //復元された３次元点群の値を受け取る配列のポインタ
              scale,      //移動量
              focal,      //焦点距離
              cx,         //画像中心ｘ
              cy,         //画像中心ｙ
              point_count,//対応が付いている特徴点の数
              u1,         //１枚目の画像の特徴点ｘ座標配列のポインタ
              v1,         //１枚目の画像の特徴点ｙ座標配列のポインタ
              u2,         //２枚目の画像の特徴点ｘ座標配列のポインタ
              v2          //２枚目の画像の特徴点ｙ座標配列のポインタ
  );

  //R行列の推定値を表示
  printf("R=\n%12.5f %12.5f %12.5f\n",  Rret[0], Rret[1], Rret[2]);
  printf("%12.5f %12.5f %12.5f\n",      Rret[3], Rret[4], Rret[5]);
  printf("%12.5f %12.5f %12.5f\n\n",    Rret[6], Rret[7], Rret[8]);

  //Tベクトルの推定値を表示
  printf("T=\n%12.5f %12.5f %12.5f\n\n",Tret[0], Tret[1], Tret[2]);

  //３次元点の元の座標と推定した座標を交互に表示
  for (int i=0; i<point_count; i++){
    printf("%04d:Org %12.5f %12.5f %12.5f\n",i,point3d1[i].x, point3d1[i].y, point3d1[i].z);
    printf("%04d:Est %12.5f %12.5f %12.5f\n",i,P3D[i*3+0], P3D[i*3+1], P3D[i*3+2]);
  }
}
