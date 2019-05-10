/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef GYMPP_COMMON
#define GYMPP_COMMON

#include <any>
#include <memory>
#include <optional>
#include <typeinfo>
#include <variant>
#include <vector>

namespace gympp {

    using DataSupport = double;
    struct Range;

    template <typename Type>
    struct BufferContainer
    {
        typedef std::vector<Type> type;
    };

    using BufferInt = BufferContainer<int>::type;
    using BufferLong = BufferContainer<size_t>::type;
    using BufferFloat = BufferContainer<float>::type;
    using BufferDouble = BufferContainer<double>::type;
    using GenericBuffer = std::variant<BufferInt, BufferLong, BufferFloat, BufferDouble>;

    namespace data {
        using Shape = std::vector<size_t>;

        struct Sample;
    } // namespace data
} // namespace gympp

struct gympp::data::Sample
{
    GenericBuffer buffer;

    Sample() = default;

    Sample(const BufferInt& buf)
        : buffer(buf)
    {}
    Sample(const BufferLong& buf)
        : buffer(buf)
    {}
    Sample(const BufferFloat& buf)
        : buffer(buf)
    {}
    Sample(const BufferDouble& buf)
        : buffer(buf)
    {}

    template <typename T>
    std::optional<T> get(const size_t i) const
    {
        auto bufferPtr = std::get_if<typename BufferContainer<T>::type>(&buffer);

        if (!bufferPtr) {
            return {};
        }

        if (i >= bufferPtr->size()) {
            return {};
        }

        return bufferPtr->at(i);
    }

    template <typename T>
    const std::vector<T>* getBuffer() const
    {
        return std::get_if<typename BufferContainer<T>::type>(&buffer);
    }

    template <typename T>
    std::vector<T>* getBuffer()
    {
        return std::get_if<typename BufferContainer<T>::type>(&buffer);
    }
};

struct gympp::Range
{
    Range(DataSupport minValue = std::numeric_limits<DataSupport>::lowest(),
          DataSupport maxValue = std::numeric_limits<DataSupport>::max())
        : min(minValue)
        , max(maxValue)
    {}

    DataSupport min = 0;
    DataSupport max = 0;

    bool contains(double value) { return (value <= max && value >= min) ? true : false; }
};

#endif // GYMPP_COMMON
