package zap

import (
	"os"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg/logger/config"
	"go.uber.org/zap"
	"go.uber.org/zap/zapcore"
)

func New(cfg config.Configuration) (*zap.Logger, error) {

	logLevel := setLogLevel(cfg.Level)

	encoderCfg := zapcore.EncoderConfig{
		TimeKey:        "timestamp",
		LevelKey:       "level",
		NameKey:        "logger",
		CallerKey:      "caller",
		MessageKey:     "msg",
		StacktraceKey:  "stacktrace",
		LineEnding:     zapcore.DefaultLineEnding,
		EncodeLevel:    zapcore.LowercaseLevelEncoder,
		EncodeTime:     zapcore.TimeEncoder(timeEncoder(cfg.TimeFormat)),
		EncodeDuration: zapcore.SecondsDurationEncoder,
		EncodeCaller:   zapcore.ShortCallerEncoder,
	}

	log := zap.New(zapcore.NewCore(zapcore.NewConsoleEncoder(encoderCfg), zapcore.AddSync(os.Stdout), logLevel))

	log.Sync()
	return log, nil
}

func setLogLevel(logLevel int) zap.AtomicLevel {
	atom := zap.NewAtomicLevel()

	switch logLevel {
	case config.FATAL_LEVEL:
		atom.SetLevel(zapcore.FatalLevel)
	case config.ERROR_LEVEL:
		atom.SetLevel(zapcore.ErrorLevel)
	case config.WARN_LEVEL:
		atom.SetLevel(zapcore.WarnLevel)
	case config.INFO_LEVEL:
		atom.SetLevel(zapcore.InfoLevel)
	case config.DEBUG_LEVEL:
		atom.SetLevel(zapcore.DebugLevel)
	default:
		atom.SetLevel(zapcore.InfoLevel)
	}

	return atom
}

func timeEncoder(format string) func(time.Time, zapcore.PrimitiveArrayEncoder) {
	return func(t time.Time, enc zapcore.PrimitiveArrayEncoder) {
		enc.AppendString(t.Format(format))
	}
}
