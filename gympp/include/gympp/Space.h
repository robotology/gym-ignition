/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef GYMPP_SPACES_SPACE
#define GYMPP_SPACES_SPACE

#include "gympp/Common.h"

#include <functional>
#include <memory>

namespace gympp {
    namespace spaces {
        class Box;
        class Space;
        class Discrete;
        using SpacePtr = std::shared_ptr<Space>;
    } // namespace spaces
} // namespace gympp

class gympp::spaces::Space
{
public:
    using Sample = gympp::data::Sample;

    Space() = default;
    virtual ~Space() = default;

    // Uniformly randomly sample a random element of this space
    virtual Sample sample() = 0;

    // Returns true if data is a valid member of this space
    virtual bool contains(const Sample& data) const = 0;

    // TODO:
    // to_jsonable
    // from_jsonable
};

class gympp::spaces::Box : public gympp::spaces::Space
{
public:
    using Shape = gympp::data::Shape;
    using Buffer = typename gympp::BufferContainer<DataSupport>::type;
    using Limit = Buffer;
    using Sample = gympp::data::Sample;

    Box() = delete;
    Box(const DataSupport low, const DataSupport high, const Shape& shape);
    Box(const Limit& low, const Limit& high);
    ~Box() override = default;

    Sample sample() override;
    bool contains(const Sample& data) const override;

    Limit high();
    Limit low();
    Shape shape();

private:
    class Impl;
    std::unique_ptr<Impl, std::function<void(Impl*)>> pImpl;
};

class gympp::spaces::Discrete : public gympp::spaces::Space
{
public:
    using Type = int;
    using Shape = gympp::data::Shape;

    Discrete() = delete;
    Discrete(size_t n);
    ~Discrete() override = default;

    Sample sample() override;
    bool contains(const Sample& data) const override;

    size_t n();
    Shape shape();

private:
    class Impl;
    std::unique_ptr<Impl, std::function<void(Impl*)>> pImpl;
};

#endif // GYMPP_SPACES_SPACE
