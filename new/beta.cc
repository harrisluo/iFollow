#include <iostream>
#include <string>
#include <vector>

int main2() {
    std::string s;
    std::getline(std::cin, s);
    if (std::cin.eof()) {
        std::cout << s << std::endl;
        std::cout << "EOF" << std::endl;
    }

    std::getline(std::cin, s);
    std::cout << s;
    std::cout << '!' << std::endl;
    s = "你";
    std::cout << s.length() << std::endl;
    return 0;
}

int main() {
    std::cout << isprint((char)999) << std::endl;
    std::cout << (char)999 << std::endl;
    std::vector<int> vec = { 1, 2, 3 };
    std::cout << vec[1] << std::endl;
    return 0;
}