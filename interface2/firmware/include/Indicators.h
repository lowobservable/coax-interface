#pragma once

typedef enum {
    UNKNOWN,
    CONFIGURING,
    RUNNING
} status_t;

class Indicators
{
public:
    Indicators(const int statusPin, const int txPin, const int rxPin,
            const int errorPin);

    void begin();

    void setStatus(const status_t status);

    void error();

    void refresh();

private:
    int _statusPin;
    int _txPin;
    int _rxPin;
    int _errorPin;
    status_t _status;
};
