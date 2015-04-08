// Copyright (c) 2009 libmv authors.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
//
// Libmv specific init for tools. In particular, get logging and flags set up.


#ifndef LIBMV_TOOLS_TOOL_H_
#define LIBMV_TOOLS_TOOL_H_

#include <cstdio>
#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>

namespace gflags {
  using namespace google;
}

namespace libmv {

inline void Init(const char *usage, int *argc, char ***argv) {
  google::InitGoogleLogging((*argv)[0]);
  gflags::SetUsageMessage(std::string(usage));
  gflags::ParseCommandLineFlags(argc, argv, true);
}

}  // namespace libmv

#endif  // ifndef LIBMV_TOOLS_TOOL_H_
