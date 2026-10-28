// Package config provides configuration structures for the logger.
package config

import (
	"io"
	"os"
	"time"
)

// Log level constants.
const (
	FatalLevel int = iota
	ErrorLevel
	WarnLevel
	InfoLevel
	DebugLevel
)

// Configuration represents logger configuration options.
type Configuration struct {
	Writer     io.Writer
	TimeFormat string
	Level      int
}

// Validate validates and populates default settings for the configuration.
func (c *Configuration) Validate() error {
	if c.Writer == nil {
		c.Writer = os.Stdout
	}

	if c.TimeFormat == "" {
		c.TimeFormat = time.RFC3339Nano
	}

	return nil
}
