%{
#include <optional>
%}

%naturalvar;

namespace std {
    template <class T>
    class optional {
    public:
        typedef T value_type;
    
        optional();
        optional(const optional&);
        optional(const T&);
        
        bool has_value() const;
        T value();
    };
}
