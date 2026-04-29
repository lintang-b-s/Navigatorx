// Package config provides configuration structures for the logger.
package config

import (
	"io"
	"os"
	"time"
)

const (
	FatalLevel int = iota
	ErrorLevel
	WarnLevel
	InfoLevel
	DebugLevel
)

type Configuration struct {
	Writer     io.Writer
	TimeFormat string
	Level      int
}

func (c *Configuration) Validate() error {
	if c.Writer == nil {
		c.Writer = os.Stdout
	}

	if c.TimeFormat == "" {
		c.TimeFormat = time.RFC3339Nano
	}

	return nil
}
