#include "FirmwareUpdater.h"


void FirmwareUpdater::update()
{
    if (!HexTransfer::is_transfer_in_progress()) {
        Serial.println("No transfer in progress.");
    }
    else {
        Serial.println("Transfer in progress...");
    }
    if (HexTransfer::is_file_transfer_complete()) {
        Serial.println("File transfer complete.");
    }
}

