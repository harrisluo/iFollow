#include <iostream>
#include <fstream>

int main( int argc, char * argv[] ) {
    std::ifstream input;
    input.open(argv[1]);
    if (input) {
        std::cout << "SUCCESS" << std::endl;
    } else {
        std::cout << "FAIL" << std::endl;
    }
    return 0;
}