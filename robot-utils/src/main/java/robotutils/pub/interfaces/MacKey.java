package robotutils.pub.interfaces;

import java.util.Arrays;


/** Value-object key for MAC suffix bytes with content-based equality. */
public class MacKey {
    private final int[] m_suffixBytes;

    /** Constructor. */
    public MacKey(int... suffixBytes) {
        if (suffixBytes == null || suffixBytes.length == 0) {
            throw new IllegalArgumentException("suffixBytes must contain at least one byte");
        }
        m_suffixBytes = Arrays.copyOf(suffixBytes, suffixBytes.length);
    }

    /** Returns a defensive copy of the MAC suffix bytes. */
    public int[] suffixBytes() {
        return Arrays.copyOf(m_suffixBytes, m_suffixBytes.length);
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (!(obj instanceof MacKey other)) {
            return false;
        }
        return Arrays.equals(m_suffixBytes, other.m_suffixBytes);
    }

    @Override
    public int hashCode() {
        return Arrays.hashCode(m_suffixBytes);
    }
}
