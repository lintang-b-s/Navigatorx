package config

import (
	"io"
	"os"
	"time"
)

const (
	FATAL_LEVEL int = iota
	ERROR_LEVEL
	WARN_LEVEL
	INFO_LEVEL
	DEBUG_LEVEL
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
