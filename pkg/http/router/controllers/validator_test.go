package controllers

import (
	"testing"

	"github.com/go-playground/locales/en"
	ut "github.com/go-playground/universal-translator"
	"github.com/go-playground/validator/v10"
	enTranslations "github.com/go-playground/validator/v10/translations/en"
	"github.com/stretchr/testify/assert"
)

func TestTranslateError(t *testing.T) {
	validate := validator.New()
	english := en.New()
	uni := ut.New(english, english)
	trans, _ := uni.GetTranslator("en")
	_ = enTranslations.RegisterDefaultTranslations(validate, trans)

	type TestStruct struct {
		Field string `validate:"required"`
	}

	t.Run("Nil Error", func(t *testing.T) {
		errs := translateError(nil, trans)
		assert.Nil(t, errs)
	})

	t.Run("Validation Error", func(t *testing.T) {
		s := TestStruct{}
		err := validate.Struct(s)
		assert.Error(t, err)

		errs := translateError(err, trans)
		assert.Len(t, errs, 1)
		assert.Equal(t, "Field is a required field", errs[0].Error())
	})
}
