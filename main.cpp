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

int main() {
  cv_base cb;
  //  cb.showImg();
  //  cb.rgb2gray();
  string img_path = "/home/roc/Pictures/cvt1.jpg";
  cb.linearTest(img_path, 1, 2);
  //  Mat m1 = Mat::zeros(200, 400, CV_8UC3);
  //  cb.picCreat(m1, "White");

  Mat imgOri = imread(img_path);
  if (imgOri.empty()) {
    cout << "Picture load failed!";
    return -1;
  }
  namedWindow("original");
  imshow("original", imgOri);

  cb.noisePicCreat(imgOri, "Noise", 10);
  cb.addTest(imgOri, "Noise", 10);
  // for (int i = 0; i < 10; i++)
  //	cout << BoxMuller() << endl;;

  waitKey(0);

  return 0;
}
