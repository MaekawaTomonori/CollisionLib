#include "System.h"
#include <rpc.h>

#pragma comment(lib, "rpcrt4.lib")
namespace System{
    std::string CreateUniqueId() {
    	UUID uuid;
        UuidCreate(&uuid);
        RPC_CSTR szUuid = nullptr;
        UuidToStringA(&uuid, &szUuid);
        struct UUIDCleaner{
            RPC_CSTR& ptr;
            ~UUIDCleaner() {
                if (ptr)RpcStringFreeA(&ptr);
            }
        } cleaner {szUuid};
        return reinterpret_cast<char*>(szUuid);
    }
}
