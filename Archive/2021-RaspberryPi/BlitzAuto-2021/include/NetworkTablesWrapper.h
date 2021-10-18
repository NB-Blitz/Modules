#pragma once

#include <iostream>
#include <networktables/NetworkTableInstance.h>

using namespace std;
using namespace nt;

namespace Blitz {
    class NetworkTablesWrapper
    {
        public:
            NetworkTablesWrapper();

            double GetDouble(string entry, double defaultValue);
            bool   GetBool  (string entry, bool defaultValue);

            void SetDouble(string entry, double value);
            void SetBool  (string entry, bool value);

        private:
            NetworkTableInstance NTInst;
            std::shared_ptr<NetworkTable> NTable;
    };
}