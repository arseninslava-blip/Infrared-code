#pragma once

class ExponentialFilter {
private:
    float alpha;
    float filteredValue;
    bool initialized;

public:
    ExponentialFilter(float alpha = 0.3f) : alpha(alpha), filteredValue(0), initialized(false) {}

    void setAlpha(float a) { alpha = constrain(a, 0.01f, 0.99f); }

    float filter(float value) {
        if (!initialized) {
            filteredValue = value;
            initialized = true;
        } else {
            filteredValue = alpha * value + (1.0f - alpha) * filteredValue;
        }
        return filteredValue;
    }

    void reset() { initialized = false; }
    float get() { return filteredValue; }
};

class MedianFilter {
private:
    static const int WINDOW_SIZE = 5;
    float buffer[WINDOW_SIZE];
    int index;
    int count;

public:
    MedianFilter() : index(0), count(0) {
        for (int i = 0; i < WINDOW_SIZE; i++) buffer[i] = 0;
    }

    float filter(float value) {
        buffer[index] = value;
        index = (index + 1) % WINDOW_SIZE;
        if (count < WINDOW_SIZE) count++;

        float temp[WINDOW_SIZE];
        for (int i = 0; i < count; i++) temp[i] = buffer[i];

        for (int i = 0; i < count - 1; i++) {
            for (int j = 0; j < count - i - 1; j++) {
                if (temp[j] > temp[j + 1]) {
                    float t = temp[j];
                    temp[j] = temp[j + 1];
                    temp[j + 1] = t;
                }
            }
        }

        return temp[count / 2];
    }

    void reset() { count = 0; index = 0; }
};
