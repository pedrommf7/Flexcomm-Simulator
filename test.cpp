// #include <iostream>
// #include <cmath>
// #include <ctime>
// #include <chrono>
// #include <stdlib.h>

// double log_a_to_base_b(int a, double b)
// {
//     return std::log2(a) / std::log2(b);
// }

// int main() {
//     srand (time(NULL));


//     auto start = std::chrono::high_resolution_clock::now();

//     // do something
//     for (int i=0;i<100000;i++){
//         int value = rand() % 100 + 1; // between 0 and 100

//         double x = std::sqrt(5.0*value)+3;
//     }

//     auto end = std::chrono::high_resolution_clock::now();
//     auto elapsed1 = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
//     std::cout << "Elapsed time: " << elapsed1 << " ns" << std::endl;

//     start = std::chrono::high_resolution_clock::now();

//     // do something
//     for (int i=0;i<100000;i++){
//         int value = rand() % 100 + 1; // between 0 and 100

//         double x = log_a_to_base_b(value, 1.5);
//     }

//     end = std::chrono::high_resolution_clock::now();
//     auto elapsed2 = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
//     std::cout << "Elapsed time: " << elapsed2 << " ns" << std::endl;
    
//     if (elapsed1 > elapsed2) {
//         std::cout << "log_a_to_base_b is faster" << std::endl;
//     } else {
//         std::cout << "sqrt is faster" << std::endl;
//     }
    
//     return 0;
// }


#include <iostream>
#include <chrono>
#include <cmath>
#include <ctime>



int main() {
    const int numIterations = 100000000;
    double result = 0.0; // To store the result to prevent optimization
    srand (time(NULL));

    // Measure execution time for log₁.₅(x)
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < numIterations; i++) {
        int value = rand() % 100 + 1; // between 0 and 100

        result += log2(value); // Replace 1.5 with your value of 'x'
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    // Measure execution time for √(5x) + 3
    start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < numIterations; i++) {
        int value = rand() % 100 + 1; // between 0 and 100

        result += sqrt(5.0 * value) + 3.0; // Replace 10.0 with your value of 'x'
    }
    end = std::chrono::high_resolution_clock::now();
    auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    std::cout << "Time taken for log₁.₅(x): " << duration1.count() << " microseconds\n";
    std::cout << "Time taken for √(5x) + 3: " << duration2.count() << " microseconds\n";

    // Output 'result' to prevent optimization
    std::cout << "Result: " << result << std::endl;

    if(duration1.count() > duration2.count()) {
        std::cout << "√(5x) + 3 is faster" << std::endl;
    } else {
        std::cout << "log₁.₅(x) is faster" << std::endl;
    }

    return 0;
}