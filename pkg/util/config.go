package util

import (
	"fmt"

	"github.com/spf13/viper"
)

func ReadConfig(wordkingDir string) error {
	viper.SetConfigName("default")
	viper.AddConfigPath(wordkingDir + "/data/")

	err := viper.ReadInConfig()
	if err != nil {
		return fmt.Errorf("fatal error config file: %w", err)
	}
	return nil
}

func ReadProfileConfig(wordkingDir string, profile string) error {
	viper.SetConfigName(profile)
	viper.AddConfigPath(wordkingDir + "/data/")

	err := viper.ReadInConfig()
	if err != nil {
		return fmt.Errorf("fatal error config file: %w", err)
	}
	return nil
}
