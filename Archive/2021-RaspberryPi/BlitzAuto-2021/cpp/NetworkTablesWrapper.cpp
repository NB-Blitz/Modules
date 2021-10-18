#include "NetworkTablesWrapper.h"

Blitz::NetworkTablesWrapper::NetworkTablesWrapper()
{
    NTInst = nt::NetworkTableInstance::GetDefault();
    NTable = NTInst.GetTable("SmartDashboard");

    NTInst.StartClientTeam(5148);
    NTInst.StartDSClient();
	cout << "Connected to Network Tables" << endl;
}

// Gets
double Blitz::NetworkTablesWrapper::GetDouble(string entry, double defaultValue)
{
    return NTable->GetEntry(entry).GetDouble(defaultValue);
}
bool Blitz::NetworkTablesWrapper::GetBool(string entry, bool defaultValue)
{
    return NTable->GetEntry(entry).GetBoolean(defaultValue);
}

// Sets
void Blitz::NetworkTablesWrapper::SetDouble(string entry, double value)
{
    NTable->GetEntry(entry).SetDouble(value);
}
void Blitz::NetworkTablesWrapper::SetBool(string entry, bool value)
{
    NTable->GetEntry(entry).SetBoolean(value);
}