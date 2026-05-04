// Package config berisi code untuk read .yaml configuration
package config

import (
	"fmt"
	"os"
	"path/filepath"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/spf13/viper"
)

func FindProjectWorkingDir() (string, error) {
	dir, err := os.Getwd()
	if err != nil {
		return "", err
	}
	startDir := dir
	for i := 0; i < 10; i++ {
		if _, err := os.Stat(filepath.Join(dir, "go.mod")); err == nil {
			return dir, nil
		}
		parent := filepath.Dir(dir)
		if parent == dir {
			return startDir, nil
		}
		dir = parent
	}
	return "", fmt.Errorf("working directory not found")
}

func ReadConfig(wordkingDir string) error {
	viper.SetConfigName("default")
	viper.AddConfigPath(wordkingDir + "/data/")

	err := viper.ReadInConfig()
	if err != nil {
		return fmt.Errorf("fatal error config file: %v", err)
	}
	return nil
}

func ReadProfileConfig(wordkingDir string, profile string) error {
	viper.SetConfigName(profile)
	viper.AddConfigPath(wordkingDir + "/data/")

	err := viper.ReadInConfig()
	if err != nil {
		return fmt.Errorf("fatal error config file: %v", err)
	}
	return nil
}

func InitConfig() {
	workingDir, err := FindProjectWorkingDir()
	if err != nil {
		panic(err)
	}
	err = ReadConfig(workingDir)
	if err != nil {
		panic(err)
	}
	vehicleType := viper.GetString("vehicle_type")
	pkg.VehicleType = pkg.GetVehicleType(vehicleType)
	pkg.DoubleTrackedVehicle = pkg.GetIsDoubleTrackedVehicle()
	pkg.IsVehicle = pkg.GetIsVehicle()
	pkg.MotorizedVehicle = pkg.GetIsMotorizedVehicle()
}

func InitProfileConfig(profileName string, regionName string) {
	workingDir, err := FindProjectWorkingDir()
	if err != nil {
		panic(err)
	}
	err = ReadProfileConfig(workingDir, profileName)
	if err != nil {
		panic(err)
	}
	pkg.ProfileName = profileName
	vehicleType := viper.GetString("vehicle_type")
	pkg.VehicleType = pkg.GetVehicleType(vehicleType)
	pkg.DoubleTrackedVehicle = pkg.GetIsDoubleTrackedVehicle()
	pkg.IsVehicle = pkg.GetIsVehicle()
	pkg.MotorizedVehicle = pkg.GetIsMotorizedVehicle()
	pkg.RegionName = regionName
}
