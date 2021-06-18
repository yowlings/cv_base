/******************* LICENSE AND COPYRIGHT **********************/
/*
Software License Agreement (BSD License)

Copyright (c) 2019, rocwang @ DROID All rights reserved.
Email: yowlings@droid.ac.cn.
Github: https://github.com/yowlings

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

 * Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above
   copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided
   with the distribution.
 * Neither the name of the DROID nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/
/**************** LICENSE AND COPYRIGHT END ***********************/

#include "cv_base.h"

cv_base::cv_base() {}
void cv_base::showImg() {
  Mat img = imread("/home/roc/Pictures/town.jpg"); // Put Girl.jpg into the same path with source code.
  showImgPara(img);
  namedWindow("showImg"); // Windowns named Girl
  imshow("showImg", img); // Show img in Girl window
                          // Waiting for a key input or 6S timeout
}
void cv_base::rgb2gray() {
  Mat img = imread("/home/roc/Pictures/town.jpg"); // Put Girl.jpg into the same path with source code.
  if (img.empty()) {
    cout << "The picture is not exist!" << endl;
    return;
  }
  namedWindow("Origin"); // Windowns named Girl
  imshow("Origin", img); // Show img in Girl window

  Mat gray;
  cvtColor(img, gray, COLOR_BGR2GRAY);
  namedWindow("Gray");
  imshow("Gray", gray);
  imwrite("town_gray.jpg", gray);
}

void cv_base::showImgPara(Mat &img) {
  cout << "sizeof(img) is: " << sizeof(img) << ", img size is: " << img.size << endl;
  cout << "rows x cols: (" << img.rows << " x " << img.cols << ")" << endl;
  cout << "dims: " << img.dims << endl;
  cout << "channels: " << img.channels() << endl;
  cout << "type: " << img.type() << endl;
  cout << "depth:" << img.depth() << endl;
  cout << "elemSize:" << img.elemSize() << " (Bytes per element)" << endl;
  cout << "elemSize1:" << img.elemSize1() << "(Bytes per channel)" << endl;
  cout << "step[0]: " << img.step[0] << " (Bytes per cows only when 2 dims)" << endl;
  cout << "step[1]: " << img.step[1] << " (Bytes per element only when 2 dims)" << endl;
  cout << "step1(0): " << img.step1(0) << ", step1(1): " << img.step1(1) << " (step / elemSize1)" << endl;
}

void cv_base::picCreat(Mat &img, string name) {
  for (int i = 0; i < img.rows; i++) {
    uchar *p = img.ptr<uchar>(i);
    for (int j = 0; j < img.cols * img.channels(); j += img.channels()) {
      if (i > img.rows / 4 && i < img.rows * 3 / 4 && j > img.cols * img.channels() / 4 &&
          j < img.cols * img.channels() * 3 / 4) {
        p[j] = 0;
        p[j + 1] = 0;
        p[j + 2] = 0;

      } else {
        p[j] = 255;
        p[j + 1] = 255;
        p[j + 2] = 255;
      }
    }
  }

  cout << endl << "/******************picCreat******************/" << endl;
  showImgPara(img);
  namedWindow(name);
  imshow(name, img);
}

void cv_base::linearTest(string name, float a, float b) {
  /* Original Picture */
  Mat imgOri = imread(name);
  showImgPara(imgOri);
  namedWindow("original");
  imshow("original", imgOri);

  /* Output Picture */
  Mat imgOut = Mat::ones(imgOri.size(), imgOri.type());
  // namedWindow("zeros");
  // imshow("zeros", imgOut);

  uchar bOri = 0, gOri = 0, rOri = 0;

  for (int i = 0; i < imgOri.cols; i++) {
    for (int j = 0; j < imgOri.rows; j++) {
      bOri = imgOri.at<Vec3b>(j, i)[0];
      gOri = imgOri.at<Vec3b>(j, i)[1];
      rOri = imgOri.at<Vec3b>(j, i)[2];

      if (a >= 0) {
        imgOut.at<Vec3b>(j, i)[0] = saturate_cast<uchar>(a * bOri + b);
        imgOut.at<Vec3b>(j, i)[1] = saturate_cast<uchar>(a * gOri + b);
        imgOut.at<Vec3b>(j, i)[2] = saturate_cast<uchar>(a * rOri + b);
      } else {
        imgOut.at<Vec3b>(j, i)[0] = saturate_cast<uchar>(255 + a * bOri + b);
        imgOut.at<Vec3b>(j, i)[1] = saturate_cast<uchar>(255 + a * gOri + b);
        imgOut.at<Vec3b>(j, i)[2] = saturate_cast<uchar>(255 + a * rOri + b);
      }
    }
  }

  namedWindow("output");
  imshow("output", imgOut);
}
static double BoxMuller() {
  const double epsilon = numeric_limits<double>::min();
  double U0, U1;
  double Z0, Z1;

  do {
    U0 = rand() * (1.0 / RAND_MAX);
    U1 = rand() * (1.0 / RAND_MAX);
  } while (U0 < epsilon || U1 < epsilon);

  Z0 = sqrt(-2.0 * log(U0)) * cos(2 * CV_PI * U1);
  // Z1 = sqrt(-2.0*log(U0))*sin(2 * CV_PI*U1);

  // printf("[rand]U0 = %lf, U1 = %lf, Z0 = %lf, Z1 = %lf\n\n", U0, U1, Z0, Z1);

  return Z0; // We use one of them is OK.
}

void cv_base::noisePicCreat(Mat &imgOri, string fileName, int noiseNum) {
  int row, col, rows, cols, channals;
  int noisePicCount;
  Mat *pNoisePicList = new Mat[noiseNum];

  rows = imgOri.rows;
  cols = imgOri.cols;
  channals = imgOri.channels();

  for (noisePicCount = 0; noisePicCount < noiseNum; noisePicCount++) {
    pNoisePicList[noisePicCount] = imgOri.clone();
    for (row = 0; row < rows; row++) {
      uchar *pOri = imgOri.ptr<uchar>(row);
      uchar *pNoise = pNoisePicList[noisePicCount].ptr<uchar>(row);
      for (col = 0; col < cols * channals; col++) {
        pNoise[col] = saturate_cast<uchar>(BoxMuller() * 32 + (double)pOri[col]);
        pNoise[col + 1] = saturate_cast<uchar>(BoxMuller() * 32 + (double)pOri[col + 1]);
        pNoise[col + 2] = saturate_cast<uchar>(BoxMuller() * 32 + (double)pOri[col + 2]);
      }
    }

    cout << "col = " << col << " , cols = " << cols << endl;

    namedWindow("noisePic" + to_string(noisePicCount));
    imshow("noisePic" + to_string(noisePicCount), pNoisePicList[noisePicCount]);
    imwrite(fileName + to_string(noisePicCount) + ".png", pNoisePicList[noisePicCount]);
  }

  delete[] pNoisePicList;
  cout << "-----------Done----------" << endl;
}

bool cv_base::addTest(Mat imgOri, string fileName, int noiseNumber) {
  Mat *pNoisePicList = new Mat[noiseNumber];
  Mat imgAdd = Mat::zeros(imgOri.size(), imgOri.type());

  for (int i = 0; i < noiseNumber; i++) {
    pNoisePicList[i] = imread(fileName + to_string(i) + ".png");
    if (pNoisePicList[i].empty()) {
      cout << "Cannot find Noise" << noiseNumber << ".png" << endl;
      return false;
    }
  }

  for (int j = 0; j < noiseNumber; j++) {
    addWeighted(imgAdd, 1.0, pNoisePicList[j], 1.0 / noiseNumber, 0.0, imgAdd);
  }

  namedWindow("imgAdd");
  imshow("imgAdd", imgAdd);

  delete[] pNoisePicList;
}
