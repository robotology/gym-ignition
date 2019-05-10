/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "gympp/Space.h"
#include "gympp/Log.h"
#include "gympp/Random.h"

#include <cassert>
#include <ostream>
#include <random>
#include <utility>
#include <vector>

using namespace gympp::spaces;

// ===
// BOX
// ===

class Box::Impl
{
public:
    Box::Limit low;
    Box::Limit high;
    Shape shape;
};

Box::Box(const DataSupport low, const DataSupport high, const Shape& shape)
    : pImpl{new Impl(), [](Impl* impl) { delete impl; }}
{
    // TODO
    assert(shape.size() == 1);

    pImpl->shape = shape;
    pImpl->low = Limit(shape[0], low);
    pImpl->high = Limit(shape[0], high);
}

Box::Box(const Limit& low, const Limit& high)
    : pImpl{new Impl(), [](Impl* impl) { delete impl; }}
{
    assert(low.size() == high.size());
    pImpl->shape = {low.size()};

    // TODO
    assert(pImpl->shape.size() == 1);

    pImpl->low = low;
    pImpl->high = high;
}

Box::Sample Box::Box::sample()
{
    Space::Sample randomSample;

    // Create the buffer
    // TODO: 1D
    assert(pImpl->shape.size() == 1);
    assert(pImpl->shape[0] != 0);
    auto data = Buffer(pImpl->shape[0], DataSupport{});

    // Fill it with random data within the bounds
    for (unsigned i = 0; i < data.size(); ++i) {
        auto min = pImpl->low[i];
        auto max = pImpl->high[i];

        std::uniform_real_distribution<> distr(min, max);
        data[i] = distr(Random::engine());
    }

    // Create a Sample containing the buffer
    randomSample.buffer = std::move(data);

    return randomSample;
}

bool Box::contains(const Space::Sample& data) const
{
    auto* bufferPtr = data.getBuffer<DataSupport>();

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
            gymppError << "The sample does not comply to the limits set for its space" << std::endl;
            return false;
        }
    }

    return true;
}

typename Box::Limit Box::high()
{
    return pImpl->high;
}

typename Box::Limit Box::low()
{
    return pImpl->low;
}

Box::Shape Box::shape()
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
    std::uniform_int_distribution<> distr(0, static_cast<int>(pImpl->n) - 1);

    // Create the buffer
    auto buffer = gympp::BufferContainer<Type>::type(1, Type{});

    // Fill it with data
    buffer[0] = distr(Random::engine());

    // Create a Sample containing the buffer
    randomSample.buffer = std::move(buffer);

    return randomSample;
}

bool Discrete::contains(const Space::Sample& data) const
{
    auto* bufferPtr = data.getBuffer<Type>();

    // Check the type
    if (!bufferPtr) {
        gymppError << "Failed to get the buffer or the supported type from the sample" << std::endl;
        return false;
    }

    // TODO: only 1D
    assert(pImpl->shape.size() == 1);
    if (bufferPtr->size() != pImpl->shape.size()) {
        gymppError << "The size of the buffer (" << bufferPtr->size()
                   << ") does not match with the shape of the space (" << pImpl->shape.size() << ")"
                   << std::endl;
        return false;
    }

    if ((*bufferPtr)[0] < 0 || (*bufferPtr)[0] >= pImpl->n) {
        gymppError << "The sample does not comply to the limits set for its space" << std::endl;
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
