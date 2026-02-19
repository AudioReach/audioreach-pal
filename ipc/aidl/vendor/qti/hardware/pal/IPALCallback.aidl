/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

package vendor.qti.hardware.pal;

import vendor.qti.hardware.pal.PalCallbackBuffer;
import vendor.qti.hardware.pal.PalReadWriteDoneCommand;
import vendor.qti.hardware.pal.PalReadWriteDoneResult;
import vendor.qti.hardware.pal.PalCallbackReturnData;

@VintfStability
interface IPALCallback {
    void eventCallback(in long handle, in int eventId, in int eventDataSize,
        in byte[] eventData, in long cookie);

    oneway void eventCallbackRWDone(in long handle, in int eventId,
        in int eventDataSize, in PalCallbackBuffer[] rwDonePayload, in long cookie);

    PalCallbackReturnData prepareMQForTransfer(in long handle, in long cookie);

    oneway void ssrEvent (in int ssrState, in int subsystem, in long cookie);
}
