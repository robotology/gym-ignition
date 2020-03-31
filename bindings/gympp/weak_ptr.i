%{
#include <memory>
%}

%naturalvar;

// From https://stackoverflow.com/questions/24803149/how-to-use-weak-ptr-in-swig
namespace std {
template<class T> class weak_ptr {
public:
    typedef T element_type;

    weak_ptr();
    weak_ptr(const weak_ptr&);
    template<class Other>
        weak_ptr(const weak_ptr<Other>&);
    template<class Other>
        weak_ptr(const shared_ptr<Other>&);

    weak_ptr(const shared_ptr<T>&);


    void swap(weak_ptr&);
    void reset();

    long use_count() const;
    bool expired() const;
    shared_ptr<T> lock() const;
};
}
