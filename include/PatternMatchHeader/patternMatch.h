#ifndef PATTERN_MATCH_H
#define PATTERN_MATCH_H

#include "../includes.h"

/**
 * @class PatternMatch
 * Provides methods for pattern matching in a given text using the Knuth-Morris-Pratt (KMP) algorithm.
 */
class PatternMatch {
public:
    PatternMatch() = default;
    ~PatternMatch() = default;

    /**
     * Searches for all occurrences of a given pattern within a text using the Knuth-Morris-Pratt (KMP) algorithm.
     *
     * @param text The text in which to search for the pattern.
     * @param pattern The pattern to search for within the text.
     * @return A vector of integers representing the starting indices of each occurrence of the pattern in the text.
     *
     * Complexity: O(N + M) where N is the length of the text and M is the length of the pattern.
     */
    std::vector<int> KMP(const std::string& text, const std::string& pattern);

    /**
     * Benchmarks the execution time of a pattern matching algorithm for a given text and pattern.
     *
     * @param algorithm A function pointer to the pattern matching algorithm to benchmark (e.g., KMP).
     * @param text The text in which to search for the pattern.
     * @param pattern The pattern to search for within the text.
     */
    void benchmark(std::function<std::vector<int>(const std::string&, const std::string&)>, const std::string&, const std::string&);

private:
    /**
     * Computes the Longest Prefix Suffix (LPS) array for the given pattern.
     *
     * @param pattern The pattern for which to compute the LPS array.
     * @param lps A vector to store the computed LPS values.
     *
     * Complexity: O(M) where M is the length of the pattern.
     */
    void computeLPS(const std::string& pattern, std::vector<int>& lps);
};

#endif  // PATTERN_MATCH_H