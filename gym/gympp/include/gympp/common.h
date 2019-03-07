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

    template <typename DataSupport>
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
    // TODO: others
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
    std::vector<T>* getBuffer()
    {
        return std::get_if<typename BufferContainer<T>::type>(&buffer);
    }
};

// TODO: for improving performances of the type check:
// https://cpptruths.blogspot.com/2018/11/non-colliding-efficient.html
// TODO: supported type only DOUBLE for the time being
// template <typename DataType>
// class gympp::data::Sample
//{
// private:
//    std::vector<std::any> buffer;
//    std::unique_ptr<std::type_info> typeInfo;

//    std::variant<SampleInt, SampleFloat, SampleDouble> buffer2;

// public:
//    Sample(std::type_info type)
//        : typeInfo(std::make_unique<std::type_info>(type))
//    {}
//    ~Sample();

//    std::valarray<int> get() {

//    }

//    template <typename DataType>
//    DataType& operator[](size_t idx);
//};

// namespace gympp {
//    namespace data {
//        extern template float& Sample::operator[]<float>(size_t idx);
//        extern template double& Sample::operator[]<double>(size_t idx);
//    } // namespace data
//} // namespace gympp

// TODO Move to the cpp
// template <typename DataType>
// DataType& gympp::data::Sample::operator[](size_t idx)
//{
//    std::any e = buffer[idx];
//    return std::any_cast<DataType>(e);
//}

template <typename DataSupport>
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
