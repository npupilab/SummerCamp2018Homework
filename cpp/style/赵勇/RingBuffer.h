// GSLAM - A general SLAM framework and benchmark
// Copyright 2018 PILAB Inc. All rights reserved.
// https://github.com/zdzhaoyong/GSLAM
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author:xulei   email:xlnwpu@gmail.com
//
// This is the GSLAM main API header
#ifndef PIL_RINGBUFFER_H  // NOLINT
#define PIL_RINGBUFFER_H

template <typename BufType = unsigned char>
class RingBuffer {
  explicit RingBuffer(int size = 1024)
      : data(0), dataSize(size), readIdx(0), writeIdx(0) {
    data = new BufType[size];
  }

  ~RingBuffer() {
    if (data) delete[] data;
  }

  int total() const { return dataSize; }

  int used() const {
    int tmp = writeIdx - readIdx;
    if (tmp < 0) tmp += dataSize;
    return tmp;
  }

  int left() {
    int tmp = readIdx - writeIdx;
    if (tmp < 0) tmp += dataSize;
    return tmp;
  }

  bool write(BufType* buf, int length = 1) {
    if (left() < length || length <= 0) return false;
    for (int i = 0; i < length; i++) {
      if (writeIdx >= dataSize) writeIdx -= dataSize;
      data[writeIdx++] = buf[i];
    }
    return true;
  }

  bool read(BufType* buf, int length = 1) {
    if (used() < length || length <= 0) return false;
    for (int i = 0; i < length; i++) {
      if (readIdx >= dataSize) readIdx -= dataSize;
      buf[i] = data[readIdx++];
    }
    return true;
  }

 private:
  BufType* data;
  int dataSize, readIdx, writeIdx;
};
#endif  // PIL_RINGBUFFER_H  //NOLINT
