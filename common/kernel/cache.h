#ifndef CACHE_H
#define CACHE_H

/* Allows holding unitialized objects. Requires move or copy constructor.
 * Will panic in debug mode if trying to acces unitiialized object. */
template <typename T>
class Maybe {
  public:
    Maybe() : nothing(0), contains(false) {}
    Maybe(T && value) : something(std::move(value)), contains(true) {}
    Maybe(Maybe<T>&& other) {
        if (other.is_some()) {
            other.contains = false;
            new(&this->something) T(std::move(other.something));
        } else {
            this->contains = false;
        }
    }
    Maybe(const Maybe<T>& other) {
        if (this->is_some())
            this->something.~T();
        if (other.is_some()) {
            new(&this->something) T(other.something);
            this->contains = true;
        } else {
            this->contains = false;
        }
    }
    Maybe operator=(const Maybe<T>& other) {
        if (this->is_some())
            this->something.~T();
        if (other.is_some()) {
            this->something = other.something;
            this->contains = true;
        } else {
            this->contains = false;
        }
    }
    ~Maybe() {
        if (this->is_some())
            this->something.~T();
    }
    
    inline bool is_some() {
        return this->contains;
    }
    T *get() {
#ifdef DEBUG_SITE_ARCH
        assert(this->contains);
#endif
        return &this->something;
    }
  private:
    union {
        T something;
        char nothing;
    };
    bool contains = false;
};

/* Generic, static cache. Builds V from T (move) if not cached, otherwise
 * returns V that was already cached. Cache entries are associated with
 * H::hashable(T), which should build a hashable identifier for T. */
template <typename T, typename V, typename H>
class GenericStaticCache {
  public:
  
    V *get(const T& k) {
        auto hashable  = H::hashable(k);
        auto it = this->_cache.find(hashable);
        if (it == this->_cache.end()) {
            auto result = this->_cache.emplace(hashable, Maybe<V>(V(k)));
            assert(result.second);
            return result.first->second.get();
        }
        return it->second.get();
    }

    static GenericStaticCache<T, V, H>& instance() {
        static GenericStaticCache<T, V, H> instance;
        return instance;
    }

  private:
    std::unordered_map<decltype(H::hashable(*((T*)nullptr))), Maybe<V>, H> _cache;
};

#endif
