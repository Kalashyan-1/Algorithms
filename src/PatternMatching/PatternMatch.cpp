#include "../../include/PatternMatchHeader/patternMatch.h"

std::vector<int> PatternMatch::KMP(const std::string& text, const std::string& pattern) {
    int textLen = text.size();
    int patternLen = pattern.size();

    std::vector<int> res;

    std::vector<int> lps(patternLen, 0);
    computeLPS(pattern, lps);

    int i = 0;
    int j = 0;

    while (i < textLen) {
        if (pattern[j] == text[i]) {
            j++;
            i++;
        }

        if (j == patternLen) {
            res.push_back(i - j);
            j = lps[j - 1];
        } else if (i < textLen && pattern[j] != text[i]) {
            if (j != 0) {
                j = lps[j - 1];
            } else {
                i++;
            }
        }
    }
    return res;
}

void PatternMatch::computeLPS(const std::string& pattern, std::vector<int>& lps) { 
    int len = 0;
    int i = 1;
    while (i < pattern.size()) {
        if (pattern[i] == pattern[len]) {
            len++;
            lps[i] = len;
            i++;
        } else {
            if (len != 0) {
                len = lps[len - 1];
            } else {
                lps[i] = 0;
                i++;
            }
        }
    }
}


void PatternMatch::benchmark(std::function<std::vector<int>(const std::string&, const std::string&)> algorithm, const std::string& text, const std::string& pattern) {
    auto start = std::chrono::steady_clock::now();
    auto res = algorithm(text, pattern);
    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    std::cout << "Algorithm execution time: " << duration.count() << " microseconds\n";
}
