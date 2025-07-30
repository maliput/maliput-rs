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

/// Error types for maliput lib.
#[derive(Debug, PartialEq, Eq, thiserror::Error)]
pub enum MaliputError {
    // TODO(francocipollone): Add more specific error types as we handle it from the C++ side.
    #[error("Maliput assertion error: {0}")]
    AssertionError(String),
    #[error("Other: {0}")]
    Other(String),
}

impl From<cxx::Exception> for MaliputError {
    fn from(e: cxx::Exception) -> Self {
        let msg = e.to_string();
        if msg.contains("maliput::common::assertion_error") {
            MaliputError::AssertionError(msg)
        } else {
            MaliputError::Other(msg)
        }
    }
}

/// Log levels for the under the hood maliput library's logging system.
///
/// See https://github.com/maliput/maliput/blob/main/include/maliput/common/logger.h
/// for more details on the logging system and available log levels.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LogLevel {
    Off,
    Trace,
    Debug,
    Info,
    Warn,
    Error,
    Critical,
    Unchanged,
}

impl From<LogLevel> for &'static str {
    fn from(level: LogLevel) -> Self {
        match level {
            LogLevel::Off => "off",
            LogLevel::Trace => "trace",
            LogLevel::Debug => "debug",
            LogLevel::Info => "info",
            LogLevel::Warn => "warn",
            LogLevel::Error => "error",
            LogLevel::Critical => "critical",
            LogLevel::Unchanged => "unchanged",
        }
    }
}
impl From<String> for LogLevel {
    fn from(level: String) -> Self {
        match level.as_str() {
            "off" => LogLevel::Off,
            "trace" => LogLevel::Trace,
            "debug" => LogLevel::Debug,
            "info" => LogLevel::Info,
            "warn" => LogLevel::Warn,
            "error" => LogLevel::Error,
            "critical" => LogLevel::Critical,
            "unchanged" => LogLevel::Unchanged,
            _ => panic!("Invalid log level: {}", level),
        }
    }
}
impl std::fmt::Display for LogLevel {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LogLevel::Off => write!(f, "off"),
            LogLevel::Trace => write!(f, "trace"),
            LogLevel::Debug => write!(f, "debug"),
            LogLevel::Info => write!(f, "info"),
            LogLevel::Warn => write!(f, "warn"),
            LogLevel::Error => write!(f, "error"),
            LogLevel::Critical => write!(f, "critical"),
            LogLevel::Unchanged => write!(f, "unchanged"),
        }
    }
}

/// Set the log level for the maliput library's logging system.
///
/// # Arguments
///
/// * `level` - The desired log level to set. This should be one of the variants of `LogLevel`.
///
/// # Returns
///
/// A string indicating the previous log level before the change.
///
pub fn set_log_level(level: LogLevel) -> LogLevel {
    maliput_sys::common::ffi::LOG_set_log_level(level.into()).into()
}

#[cfg(test)]
mod tests {
    use super::set_log_level;
    use super::LogLevel;
    #[test]
    fn test_log_level() {
        set_log_level(LogLevel::Off);
        let last_level = set_log_level(LogLevel::Trace);
        assert_eq!(last_level, LogLevel::Off);
        let last_level = set_log_level(LogLevel::Debug);
        assert_eq!(last_level, LogLevel::Trace);
        let last_level = set_log_level(LogLevel::Info);
        assert_eq!(last_level, LogLevel::Debug);
        let last_level = set_log_level(LogLevel::Warn);
        assert_eq!(last_level, LogLevel::Info);
        let last_level = set_log_level(LogLevel::Error);
        assert_eq!(last_level, LogLevel::Warn);
        let last_level = set_log_level(LogLevel::Critical);
        assert_eq!(last_level, LogLevel::Error);
        let last_level = set_log_level(LogLevel::Unchanged);
        assert_eq!(last_level, LogLevel::Critical);
        let last_level = set_log_level(LogLevel::Off);
        assert_eq!(last_level, LogLevel::Critical);
        // TODO(francocipollone): Test an invalid log level. It throws under the hood in c++.
    }
}
