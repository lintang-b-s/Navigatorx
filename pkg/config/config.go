// Package config berisi code untuk read .yaml configuration
package config

import (
	"fmt"
	"log"
	"os"
	"path/filepath"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/spf13/viper"
)

// FindProjectWorkingDir finds the root directory of the project containing go.mod.
func FindProjectWorkingDir() (string, error) {
	dir, err := os.Getwd()
	if err != nil {
		return "", err
	}
	startDir := dir
	for i := 0; i < 4; i++ {
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

// ReadConfig reads the default configuration from the working directory.
func ReadConfig(workingDir string) error {
	viper.SetConfigName("default")
	viper.AddConfigPath(workingDir + "/data/")

	pkg.WorkingDir = workingDir

	err := viper.ReadInConfig()
	if err != nil {
		return fmt.Errorf("fatal error config file: %v", err)
	}
	return nil
}

// ReadProfileConfig reads configuration for a specific profile from the working directory.
func ReadProfileConfig(workingDir string, profile string) error {
	viper.SetConfigName(profile)
	viper.AddConfigPath(workingDir + "/data/")

	pkg.WorkingDir = workingDir

	err := viper.ReadInConfig()
	if err != nil {
		return fmt.Errorf("fatal error config file: %v", err)
	}
	return nil
}

// InitConfig initializes the default configuration settings.
func InitConfig() {
	workingDir, err := FindProjectWorkingDir()
	if err != nil {
		log.Fatal(err)
	}
	err = ReadConfig(workingDir)
	if err != nil {
		log.Fatal(err)
	}
	vehicleType := viper.GetString("vehicle_type")
	pkg.VehicleType = pkg.GetVehicleType(vehicleType)
	pkg.DoubleTrackedVehicleEnabled = pkg.GetIsDoubleTrackedVehicle()
	pkg.IsVehicleEnabled = pkg.GetIsVehicle()
	pkg.MotorizedVehicleEnabled = pkg.GetIsMotorizedVehicle()
}

// InitProfileConfig initializes configuration settings for a given profile, region, and file type.
func InitProfileConfig(profileName string, regionName string, fileType pkg.FILE_TYPE) {
	workingDir, err := FindProjectWorkingDir()
	if err != nil {
		log.Fatal(err)
	}
	err = ReadProfileConfig(workingDir, profileName)
	if err != nil {
		log.Fatal(err)
	}
	pkg.ProfileName = profileName
	vehicleType := viper.GetString("vehicle_type")
	pkg.VehicleType = pkg.GetVehicleType(vehicleType)
	pkg.DoubleTrackedVehicleEnabled = pkg.GetIsDoubleTrackedVehicle()
	pkg.IsVehicleEnabled = pkg.GetIsVehicle()
	pkg.MotorizedVehicleEnabled = pkg.GetIsMotorizedVehicle()
	pkg.RegionName = regionName
	pkg.TIPE = fileType
}

// InitRegionName initializes region settings with the default car profile.
func InitRegionName(regionName string, fileType pkg.FILE_TYPE) {
	pkg.ProfileName = "car"
	pkg.RegionName = regionName
	pkg.TIPE = fileType
}

// ProfilesRoot returns the profiles directory path based on the current execution mode.
func ProfilesRoot() string {
	var root string
	switch pkg.TIPE {
	case pkg.ROUTER:
		root = "./data/profiles"
	case pkg.EVAL:
		root = "./data/evals/profiles"
	case pkg.TEST:
		root = "./data/tests/profiles"
	default:
		return ""
	}

	return root
}
