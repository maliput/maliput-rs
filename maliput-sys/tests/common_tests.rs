// BSD 3-Clause License
//
// Copyright (c) 2025, Woven by Toyota.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#[cfg(test)]
mod common_test {
    use maliput_sys::common::ffi::LOG_set_log_level;
    #[test]
    fn test_log_level() {
        LOG_set_log_level("off");
        let last_level = LOG_set_log_level("trace");
        assert_eq!(last_level, "off");
        let last_level = LOG_set_log_level("debug");
        assert_eq!(last_level, "trace");
        let last_level = LOG_set_log_level("info");
        assert_eq!(last_level, "debug");
        let last_level = LOG_set_log_level("warn");
        assert_eq!(last_level, "info");
        let last_level = LOG_set_log_level("error");
        assert_eq!(last_level, "warn");
        let last_level = LOG_set_log_level("critical");
        assert_eq!(last_level, "error");
        let last_level = LOG_set_log_level("unchanged");
        assert_eq!(last_level, "critical");
        // TODO(francocipollone): Test an invalid log level. It throws under the hood in c++.
    }
}
