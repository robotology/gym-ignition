#ifndef GYMPP_SPACES_SPACE
#define GYMPP_SPACES_SPACE

#include "gympp/common.h"

#include <any>
#include <functional>
#include <memory>

namespace gympp {
    namespace spaces {

        namespace details {
            template <typename DataType>
            class TBox;
        }

        class Space;
        class Discrete;
        using Box = gympp::spaces::details::TBox<double>;
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

template <typename DataType>
class gympp::spaces::details::TBox : public gympp::spaces::Space
{
public:
    using Shape = gympp::data::Shape;
    using Buffer = typename gympp::BufferContainer<DataType>::type;
    using Limit = Buffer;
    using Sample = gympp::data::Sample;

    TBox() = delete;
    TBox(const DataType low, const DataType high, const Shape& shape);
    TBox(const Limit& low, const Limit& high);
    ~TBox() override = default;

    Sample sample() override;
    bool contains(const Sample& data) const override;

    Limit high();
    Limit low();
    Shape shape();

private:
    class Impl;
    std::unique_ptr<Impl, std::function<void(Impl*)>> pImpl;
};

// TODO: export the symbol and instantiate in the cpp
extern template class gympp::spaces::details::TBox<double>;

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
