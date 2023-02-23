#ifndef BIT_VECTOR_H
#define BIT_VECTOR_H

#include "nextpnr_types.h"

#include <vector>
#include <cstdint>
#include <initializer_list>

#include <cstring>

NEXTPNR_NAMESPACE_BEGIN

// =============================================================================

class BitVector;

class NegBitVector {
public:
    operator BitVector ();

private:
    NegBitVector(const BitVector& v) : bv(v) {};
    NegBitVector(NegBitVector&& other) = default;
    NegBitVector& operator = (const NegBitVector& other) = delete;
    NegBitVector& operator = (NegBitVector&& other) = delete;

    const BitVector& bv;

    friend class BitVector;
};

class BitVector {
public:

    BitVector (size_t size = 0);

    BitVector (const BitVector&) = default;

    // ................................

    // Returns size in bits
    inline size_t size () const {
        return m_Size;
    }
    // Resizes the vector
    void resize (size_t size);

    // Clears all bits
    BitVector& clear ();
    // Sets all bits
    BitVector& set ();

    // Sets a specific bit
    BitVector& setBit (size_t bit, bool value);
    bool getBit (size_t bit) const;

    // Converts to a string (for debugging)
    std::string toString () const;

    // ................................

    BitVector operator | (const BitVector& other) const;
    BitVector operator & (const BitVector& other) const;

    // Bitwise or in place
    BitVector& operator |= (const BitVector& other);
    // Bitwise and in place
    BitVector& operator &= (const BitVector& other);
    BitVector& operator &= (const NegBitVector& other);

    NegBitVector operator ~ () const {
        return NegBitVector(*this);
    }

    // Performs bitwise AND and checks if the result is non-zero
    static bool andAny (std::initializer_list<const BitVector *> vl);
    static bool andAny (const BitVector& a, const BitVector& b);

    bool any();

    class ShiftIter {
    private:
        friend class BitVector;

        const BitVector *bv;
        uint64_t window_snap;
        size_t window_shift;
        uint8_t bit_shift;
        bool end = false;

        ShiftIter(const BitVector *bitvec, bool make_end)
            : bv(bitvec)
            , window_snap(0)
            , window_shift(0)
            , bit_shift(0)
            , end((bitvec->size() == 0) || make_end)
        {
            if (!this->end)
                this->window_snap = this->bv->m_Data[0];
            
            this->operator++();
        }

        // TODO: On platforms which support it, this could be replaced by an `fls` call
        // which can force use of specialized instructions for performing that.
        static inline int8_t hibit(uint64_t v) {
            v |= (v >>  1);
            v |= (v >>  2);
            v |= (v >>  4);
            v |= (v >>  8);
            v |= (v >> 16);
            v |= (v >> 32);
            return (int8_t)(v - (v >> 1));
        }

    public:
        inline size_t operator * () {
            return this->window_shift * sizeof(uint64_t) * 8 + this->bit_shift - 1;
        }

        inline void operator ++ () {
            while (true) {
                if (this->end)
                    return;
                
                uint64_t window_shifted = window_snap >> this->bit_shift;
                int8_t bit = ShiftIter::hibit(window_shifted & ~(window_shifted - 1));
                
                if (bit == -1) {
                    this->window_shift++;
                    if (this->window_shift >= this->bv->m_Data.size()) {
                        this->end = true;
                        return;
                    }
                    this->window_snap = this->bv->m_Data[this->window_shift];
                    this->bit_shift = 0;
                    continue;
                }

                this->bit_shift += bit;

                if (this->window_shift * sizeof(uint64_t) * 8 + this->bit_shift
                    > this->bv->size())
                    this->end = true;
                
                return;
            }
        }

            

        inline bool operator == (const ShiftIter& other) {
            if (this->end && other.end)
                return true;
            
            return (this->bv == other.bv) &&
                   (this->window_shift == other.window_shift) && 
                   (this->bit_shift == other.bit_shift);
        }

        inline bool operator != (const ShiftIter& other) {
            if (!(this->end && other.end))
                return true;
            
            return (this->bv != other.bv) ||
                   (this->window_shift != other.window_shift) ||
                   (this->bit_shift != other.bit_shift);
        }
    };

    ShiftIter begin() const {
        return ShiftIter(this, false);
    }

    ShiftIter end() const {
        return ShiftIter(this, true);
    }

    // ................................

private:

    // Get raw data
    uint64_t* data () {
        return (uint64_t*)m_Data.data();
    }
    // Get raw data (const)
    const uint64_t* data () const {
        return (const uint64_t*)m_Data.data();
    }

    // ................................

    // Size (in bits)
    size_t m_Size;
    // Data
    std::vector<uint64_t> m_Data;

    friend class NegBitVector;
};

NEXTPNR_NAMESPACE_END

#endif // BIT_VECTOR_H
