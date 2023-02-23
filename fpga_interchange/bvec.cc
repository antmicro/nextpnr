#include "bvec.h"

#include <inttypes.h>

NEXTPNR_NAMESPACE_BEGIN

// =============================================================================

BitVector::BitVector (size_t size) {
    resize(size);
}

void BitVector::resize (size_t size) {
    m_Size = ((size + 63) >> 6) << 6;  // Round up 64
    m_Data.resize(m_Size >> 6);
}

// =============================================================================

BitVector& BitVector::clear () {
    std::vector<uint64_t> empty(m_Size >> 6, 0ULL);
    m_Data.swap(empty);

    return *this;
}

BitVector& BitVector::set () {
    std::vector<uint64_t> empty(m_Size >> 6, 0xFFFFFFFFFFFFFFFFULL);
    m_Data.swap(empty);

    return *this;
}

BitVector& BitVector::setBit (size_t bit, bool value) {
    NPNR_ASSERT(bit < m_Size);

    size_t   ofs = bit >> 6;            // div 64
    uint64_t msk = 1LL << (bit & 0x3F); // mod 64

    if (value) {
        m_Data[ofs] |=  msk;
    } else {
        m_Data[ofs] &= ~msk;
    }

    return *this;
}

bool BitVector::getBit (size_t bit) const {
    NPNR_ASSERT(bit < m_Size);

    size_t   ofs = bit >> 6;            // div 64
    uint64_t msk = 1LL << (bit & 0x3F); // mod 64

    return (m_Data[ofs] & msk) != 0;
}

// =============================================================================

BitVector& BitVector::operator |= (const BitVector &other) {
    NPNR_ASSERT(m_Size == other.size());

    size_t N = m_Data.size();
    uint64_t* p = data();
    const uint64_t* q = other.data();

    for (size_t i = 0; i < N; ++i) {
        *p++ |= *q++;
    }

    return *this;
}

BitVector BitVector::operator | (const BitVector& other) const {
    NPNR_ASSERT(m_Size == other.size());
    BitVector result = BitVector(other);

    result.clear();
    result |= *this;
    result |= other;

    return result;
}

BitVector BitVector::operator & (const BitVector& other) const {
    NPNR_ASSERT(m_Size == other.size());

    BitVector result = BitVector(other);

    result.set();
    result &= *this;
    result &= other;

    return result;
}

BitVector& BitVector::operator &= (const BitVector& other) {
    NPNR_ASSERT(m_Size == other.size());

    size_t N = m_Data.size();
    uint64_t* p = data();
    const uint64_t* q = other.data();

    for (size_t i = 0; i < N; ++i) {
        *p++ &= *q++;
    }

    return *this;
}


BitVector& BitVector::operator &= (const NegBitVector& other) {
    NPNR_ASSERT(m_Size == other.bv.size());

    size_t N = m_Data.size();
    uint64_t* p = data();
    const uint64_t* q = other.bv.data();

    for (size_t i = 0; i<N; ++i) {
        *p++ &= ~*q++;
    }

    return *this;
}

bool BitVector::andAny (const BitVector& a, const BitVector& b) {
    NPNR_ASSERT(a.size() == b.size());

    size_t N = a.m_Data.size();
    const uint64_t* p = a.data();
    const uint64_t* q = b.data();

    for (size_t i = 0; i < N; ++i) {
        if (*p++ & *q++) return true;
    }

    return false;
}

bool BitVector::andAny (std::initializer_list<const BitVector *> vl) {
    const BitVector *first_vec = *vl.begin();

    for (auto v : vl) {
        NPNR_ASSERT(v->size() == first_vec->size());
    }

    size_t N = first_vec->m_Data.size();
    for (size_t i = 0; i < N; ++i) {
        uint64_t tmp = 0xFFFFFFFFFFFFFFFF;
        for (auto v: vl) {
            const uint64_t* data = v->data();
            tmp &= data[i];
        }
        if (tmp) return true;
    }

    return false;
}

bool BitVector::any () {
    size_t N = this->m_Data.size();
    const uint64_t* p = this->data();

    for (size_t i = 0; i < N; ++i) {
        if (*p++) return true;
    }

    return false;
}

// =============================================================================

std::string BitVector::toString () const {
    std::string str = "[" + std::to_string(m_Size) + "] ";
    char cstr[20];

    for (size_t i=0; i<m_Data.size(); ++i) {
        size_t j = m_Data.size() - 1 - i;
        sprintf(cstr, "%016" PRIX64, m_Data[j]);
        str += std::string(cstr);
    }

    return str;
}

NegBitVector::operator BitVector () {
    BitVector bitvec(bv.size());

    size_t N = bv.size();
    uint64_t* p = bitvec.data();
    const uint64_t* q = bv.data();

    for (size_t i = 0; i < N; ++i) {
        *p++ = ~*q++;
    }

    return bitvec;
}

NEXTPNR_NAMESPACE_END
