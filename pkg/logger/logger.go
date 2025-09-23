package logger

import (
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg/logger/config"
	myZap "github.com/lintang-b-s/Navigatorx/pkg/logger/zap"
	"github.com/spf13/viper"
	"go.uber.org/zap"
)

func New() (*zap.Logger, error) {
	viper.SetDefault("LOG_LEVEL", config.INFO_LEVEL)
	viper.SetDefault("LOG_TIME_FORMAT", time.RFC3339Nano)

	cfg := config.Configuration{
		Level:      viper.GetInt("LOG_LEVEL"),
		TimeFormat: viper.GetString("LOG_TIME_FORMAT"),
	}

	err := cfg.Validate()
	if err != nil {
		return nil, err
	}

	log, err := myZap.New(cfg)

	if err != nil {
		return nil, err
	}

	return log, nil
}
