#pragma once

//Data Dictionary Definition
tDDictEntry* DDef(tDDefault* df, const char* name, const char* unit, float const* var, tDVAPlace place = DVA_IO_In)
{
    return DDefFloat(df, name, unit, const_cast<float*>(var), place);
}
tDDictEntry* DDef(tDDefault* df, const char* name, const char* unit, double const* var, tDVAPlace place = DVA_IO_In)
{
    return DDefDouble(df, name, unit, const_cast<double*>(var), place);
}
tDDictEntry* DDef(tDDefault* df, const char* name, const char* unit, long long const* var, tDVAPlace place = DVA_IO_In)
{
    return DDefLLong(df, name, unit, const_cast<long long*>(var), place);
}
tDDictEntry* DDef(tDDefault* df, const char* name, const char* unit, unsigned long long const* var, tDVAPlace place = DVA_IO_In)
{
    return DDefULLong(df, name, unit, const_cast<unsigned long long*>(var), place);
}
tDDictEntry* DDef(tDDefault* df, const char* name, const char* unit, long const* var, tDVAPlace place = DVA_IO_In)
{
    return DDefLong(df, name, unit, const_cast<long*>(var), place);
}
tDDictEntry* DDef(tDDefault* df, const char* name, const char* unit, unsigned long const* var, tDVAPlace place = DVA_IO_In)
{
    return DDefULong(df, name, unit, const_cast<unsigned long*>(var), place);
}
tDDictEntry* DDef(tDDefault* df, const char* name, const char* unit, int const* var, tDVAPlace place = DVA_IO_In)
{
    return DDefInt(df, name, unit, const_cast<int*>(var), place);
}
tDDictEntry* DDef(tDDefault* df, const char* name, const char* unit, unsigned int const* var, tDVAPlace place = DVA_IO_In)
{
    return DDefUInt(df, name, unit, const_cast<unsigned int*>(var), place);
}
tDDictEntry* DDef(tDDefault* df, const char* name, const char* unit, short const* var, tDVAPlace place = DVA_IO_In)
{
    return DDefShort(df, name, unit, const_cast<short*>(var), place);
}
tDDictEntry* DDef(tDDefault* df, const char* name, const char* unit, unsigned short const* var, tDVAPlace place = DVA_IO_In)
{
    return DDefUShort(df, name, unit, const_cast<unsigned short*>(var), place);
}
tDDictEntry* DDef(tDDefault* df, const char* name, const char* unit, char const* var, tDVAPlace place = DVA_IO_In)
{
    return DDefChar(df, name, unit, const_cast<char*>(var), place);
}
tDDictEntry* DDef(tDDefault* df, const char* name, const char* unit, signed char const* var, tDVAPlace place = DVA_IO_In)
{
    return DDefChar(df, name, unit, reinterpret_cast<char*>(const_cast<signed char*>(var)), place);
}
tDDictEntry* DDef(tDDefault* df, const char* name, const char* unit, unsigned char const* var, tDVAPlace place = DVA_IO_In)
{
    return DDefUChar(df, name, unit, const_cast<unsigned char*>(var), place);
}