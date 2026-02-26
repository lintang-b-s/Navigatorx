package util

import (
	"fmt"

	"github.com/spf13/viper"
)

func ReadConfig() error {
	viper.SetConfigName("config")
	viper.AddConfigPath("./data/")

	err := viper.ReadInConfig()
	if err != nil {
		return fmt.Errorf("fatal error config file: %w", err)
	}
	return nil
}
