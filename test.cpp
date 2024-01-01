[[nodiscard]] int func(){
    return 8;
}
int main()
{
    func(); // return value is discarded, warning
    return func(); // return value is not discarded, no warning
}