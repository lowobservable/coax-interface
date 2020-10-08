#include <Arduino.h>

#include <Indicators.h>

Indicators *_indicators;

HardwareTimer timer(TIM1);

void handleTimerInterrupt()
{
    if (_indicators != NULL) {
        _indicators->refresh();
    }
}

Indicators::Indicators(const int statusPin, const int txPin, const int rxPin,
        const int errorPin) :
    _statusPin(statusPin),
    _txPin(txPin),
    _rxPin(rxPin),
    _errorPin(errorPin),
    _status(UNKNOWN)
{
}

void Indicators::begin()
{
    _indicators = this;

    pinMode(_statusPin, OUTPUT);
    pinMode(_txPin, OUTPUT);
    pinMode(_rxPin, OUTPUT);
    pinMode(_errorPin, OUTPUT);

    digitalWrite(_statusPin, HIGH);
    digitalWrite(_txPin, HIGH);
    digitalWrite(_rxPin, HIGH);
    digitalWrite(_errorPin, HIGH);

    timer.setOverflow(10, HERTZ_FORMAT);
    timer.attachInterrupt(handleTimerInterrupt);

    timer.resume();
}

void Indicators::setStatus(const status_t status)
{
    _status = status;

    if (status == CONFIGURING) {
        digitalWrite(_statusPin, HIGH);

        digitalWrite(_txPin, LOW);
        digitalWrite(_rxPin, LOW);
        digitalWrite(_errorPin, LOW);
    } else if (status == RUNNING) {
        digitalWrite(_statusPin, HIGH);
    }
}

void Indicators::tx()
{
    if (_txState == 0) {
        _txState = 2;
    }
}

void Indicators::rx()
{
    if (_rxState == 0) {
        _rxState = 2;
    }
}

void Indicators::error()
{
    if (_errorState == 0) {
        _errorState = 2;
    }
}

void Indicators::refresh()
{
    if (_status == CONFIGURING) {
        digitalWrite(_statusPin, !digitalRead(_statusPin));
    }

    if (_txState > 0) {
        digitalWrite(_txPin, _txState == 2 ? HIGH : LOW);

        _txState--;
    }

    if (_rxState > 0) {
        digitalWrite(_rxPin, _rxState == 2 ? HIGH : LOW);

        _rxState--;
    }

    if (_errorState > 0) {
        digitalWrite(_errorPin, _errorState == 2 ? HIGH : LOW);

        _errorState--;
    }
}
