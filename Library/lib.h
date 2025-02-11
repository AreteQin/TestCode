//
// Created by qin on 12/1/22.
//

#ifndef TEST_LIB_H
#define TEST_LIB_H

class lib {
    public:
    lib(int a):_a(a){};
    int _a;
    static void StaticFunc();
    void Execute();
};


#endif //TEST_LIB_H
