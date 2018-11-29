#include "gympp/spaces/Space.h"
#include "gympp/Log.h"

#include <cassert>
#include <random>

template class gympp::spaces::details::TBox<double>;

using namespace gympp::spaces;
using namespace gympp::spaces::details;

// ===
// BOX
// ===

template <typename DataType>
class TBox<DataType>::Impl
{
public:
    TBox<DataType>::Limit low;
    TBox<DataType>::Limit high;
    Shape shape;
};

template <typename DataType>
TBox<DataType>::TBox(const DataType low, const DataType high, const Shape& shape)
    : pImpl{new Impl(), [](Impl* impl) { delete impl; }}
{
    // TODO
    size_t size = shape.size();
    assert(size < 2);

    pImpl->shape = shape;
    pImpl->low = Limit(low, size);
    pImpl->high = Limit(high, size);
}

template <typename DataType>
TBox<DataType>::TBox(const Limit& low, const Limit& high)
    : pImpl{new Impl(), [](Impl* impl) { delete impl; }}
{
    assert(low.size() == high.size());
    pImpl->shape = {low.size()};

    // TODO
    size_t size = pImpl->shape.size();
    assert(size < 2);

    pImpl->low = low;
    pImpl->high = high;
}

// Box::~Box() = default;

template <typename DataType>
typename TBox<DataType>::Sample TBox<DataType>::TBox::sample()
{
    Space::Sample randomSample;

    std::random_device rd; // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> distr(0.0, 1.0); // TODO: always double

    // Create the buffer
    // TODO: 1D
    assert(pImpl->shape.size() == 1);
    assert(pImpl->shape[0] != 0);
    auto data = Buffer(DataType{}, pImpl->shape[0]);

    // Fill it with random data within the bounds
    for (unsigned i = 0; i < data.size(); ++i) {
        auto min = pImpl->low[i];
        auto max = pImpl->high[i];

        std::uniform_real_distribution<> distr(min, max);
        data[i] = distr(gen);
    }

    // Create a Sample containing the buffer
    randomSample.buffer = std::move(data);

    return randomSample;
}

template <typename DataType>
bool TBox<DataType>::contains(const Space::Sample& data) const
{
    auto* bufferPtr = data.get<DataType>();

    // Check the type
    if (!bufferPtr) {
        gymppError << "Failed to get the buffer or the supported type from the sample" << std::endl;
        return false;
    }

    // TODO: only 1D
    assert(pImpl->shape.size() == 1);

    if (bufferPtr->size() != pImpl->shape[0]) {
        gymppError << "The size of the buffer (" << bufferPtr->size()
                   << ") does not match with the shape of the space (" << pImpl->shape[0] << ")"
                   << std::endl;
        return false;
    }

    for (unsigned i = 0; i < bufferPtr->size(); ++i) {
        if (((*bufferPtr)[i] > pImpl->high[i]) || ((*bufferPtr)[i] < pImpl->low[i])) {
            gymppError() << "The sample does not comply to the limits set for its space"
                         << std::endl;
            return false;
        }
    }

    return true;
}

template <typename DataType>
typename TBox<DataType>::Limit TBox<DataType>::high()
{
    return pImpl->high;
}

template <typename DataType>
typename TBox<DataType>::Limit TBox<DataType>::low()
{
    return pImpl->low;
}

template <typename DataType>
Box::Shape TBox<DataType>::shape()
{
    return pImpl->shape;
}

// ========
// DISCRETE
// ========

class Discrete::Impl
{
public:
    size_t n = 0;
    Shape shape;
};

Discrete::Discrete(size_t n)
    : pImpl{new Impl(), [](Impl* impl) { delete impl; }}
{
    pImpl->n = n;
    pImpl->shape = {n};
}

Discrete::Sample Discrete::sample()
{
    Space::Sample randomSample;

    std::random_device rd; // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<> distr(0, pImpl->n - 1);

    // Create the buffer
    auto buffer = gympp::BufferContainer<Type>(Type{}, pImpl->n);

    // Fill it with data
    for (auto& element : buffer) {
        element = distr(gen);
    }

    // Create a Sample containing the buffer
    randomSample.buffer = std::move(buffer);

    return randomSample;
}

bool Discrete::contains(const Space::Sample& data) const
{
    auto* bufferPtr = data.get<Type>();

    // Check the type
    if (!bufferPtr) {
        gymppError << "Failed to get the buffer or the supported type from the sample" << std::endl;
        return false;
    }

    // TODO: only 1D
    assert(pImpl->shape.size() == 1);
    if (bufferPtr->size() != pImpl->shape[0]) {
        gymppError << "The size of the buffer (" << bufferPtr->size()
                   << ") does not match with the shape of the space (" << pImpl->shape[0] << ")"
                   << std::endl;
        return false;
    }

    if ((*bufferPtr)[0] < 0 || (*bufferPtr)[0] >= pImpl->n) {
        gymppError() << "The sample does not comply to the limits set for its space" << std::endl;
        return false;
    }

    return true;
}

size_t Discrete::n()
{
    return pImpl->n;
}

Discrete::Shape Discrete::shape()
{
    return pImpl->shape;
}
