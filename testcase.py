from ctypes import c_char, c_char_p, c_double, c_int, create_string_buffer, POINTER
import datetime
import platform
import time


def load_xclib():
    if platform.system() == 'Windows':
        from ctypes import WinDLL
        xclib = WinDLL(r'C:\Program Files\EPIX\XCLIB\lib\xclibw64.dll')
    else:
        from ctypes import CDLL
        xclib = CDLL('/usr/local/xclib/lib/xclib_x86_64.so')

    xclib.pxd_PIXCIopen.argtypes = [c_char_p, c_char_p, c_char_p]
    xclib.pxd_serialWrite.argtypes = [c_int, c_int, POINTER(c_char), c_int]
    xclib.pxd_serialConfigure.argtypes = [c_int, c_int, c_double, c_int, c_int, c_int, c_int, c_int, c_int]
    return xclib


def serial_command(xclib, data, response_length=0, timeout=5):
    ret = xclib.pxd_serialWrite(1, 0, data, len(data))
    if ret < 0:
        raise Exception(f'failed to send command')

    if response_length == 0:
        return

    response = create_string_buffer(response_length)
    wait_start = datetime.datetime.now(datetime.UTC)
    while True:
        available = xclib.pxd_serialRead(1, 0, None, 0)
        if available >= response_length:
            ret = xclib.pxd_serialRead(1, 0, response, response_length)
            if ret != response_length:
                raise Exception('failed to read command response')

            return response.raw

        if (datetime.datetime.now(datetime.UTC) - wait_start).total_seconds() > timeout:
            raise Exception('timeout while waiting for command response')

        time.sleep(0.001)


def wait_for_fpga_boot(xclib):
    while True:
        ret = xclib.pxd_serialWrite(1, 0, b'\x49\x50\x19', 3)
        if ret < 0:
            raise Exception(f'failed to send status query')

        wait_start = datetime.datetime.now(datetime.UTC)
        while True:
            available = xclib.pxd_serialRead(1, 0, None, 0)
            if available >= 1:
                buf = create_string_buffer(1)
                ret = xclib.pxd_serialRead(1, 0, buf, 1)
                if ret != 1:
                    raise Exception('failed to read status')

                status = buf.raw[0]

                # Ignore ACK/checksum if present to simplify testcase
                xclib.pxd_serialFlush(1, 0, 1, 1)

                if (status & 0x04) != 0:
                    return

                break

            if (datetime.datetime.now(datetime.UTC) - wait_start).total_seconds() > 1:
                raise Exception('timeout while waiting for status')

            time.sleep(0.001)


def run_testcase():
    xclib = load_xclib()

    ret = xclib.pxd_PIXCIopen(b'-CQ 8', None, b'cam2.fmt')
    if ret != 0:
        print(f'Failed to open PIXCI')
        return 1

    ret = xclib.pxd_serialConfigure(1, 0, 115200, 8, 0, 1, 0, 0, 0)
    if ret != 0:
        print(f'Failed to configure PIXCI serial')
        return 1

    xclib.pxd_serialFlush(1, 0, 1, 1)

    wait_for_fpga_boot(xclib)

    # Disable command ACK, checksum, NVM access
    serial_command(xclib, b'\x4F\x02\x50')

    # Wait for FPGA to load NUC tables
    time.sleep(5.)

    # Disable automatic gain calculation mode
    serial_command(xclib, b'\x53\x00\x03\x01\x00\x00\x50')

    # Low gain with internal triggering
    serial_command(xclib, b'\x53\x00\x03\x01\xF2\x00\x50')

    # Set digital gain to minimum value (256)
    serial_command(xclib, b'\x53\x00\x03\x01\xC6\x01\x50')
    serial_command(xclib, b'\x53\x00\x03\x01\xC7\x00\x50')

    # Disable non-uniformity corrections
    serial_command(xclib, b'\x53\x00\x03\x01\xF9\x4C\x50')

    # Set exposure time to 1 second
    serial_command(xclib, b'\x53\x00\x03\x01\xDD\x04\x50')
    serial_command(xclib, b'\x53\x00\x03\x01\xDE\x3C\x50')
    serial_command(xclib, b'\x53\x00\x03\x01\xDF\x23\x50')
    serial_command(xclib, b'\x53\x00\x03\x01\xE0\x10\x50')

    serial_command(xclib, b'\x53\x00\x03\x01\xEE\x00\x50')
    serial_command(xclib, b'\x53\x00\x03\x01\xEF\x0F\x50')
    serial_command(xclib, b'\x53\x00\x03\x01\xF0\x42\x50')
    serial_command(xclib, b'\x53\x00\x03\x01\xF1\x40\x50')

    for i in range(10):
        status = serial_command(xclib, b'\x49\x50', 1)[0]
        field_count = xclib.pxd_videoFieldCount(1)
        videostate = serial_command(xclib, b'\x53\x01\x03\x01\x05\xF9\x50', 1)[0]
        trigger = serial_command(xclib, b'\x53\x01\x03\x01\x05\xF2\x50', 1)[0]
        ccr0 = serial_command(xclib, b'\x53\x01\x03\x01\x05\x00\x50', 1)[0]
        ccr1 = serial_command(xclib, b'\x53\x01\x03\x01\x05\x01\x50', 1)[0]
        ccr2 = serial_command(xclib, b'\x53\x01\x03\x01\x05\x02\x50', 1)[0]
        print(f'loop {i}: status: {status} field count: {field_count} videostate: {videostate:08b} trigger: {trigger:08b} control registers: {ccr0:08b} {ccr1:08b} {ccr2:08b}')
        time.sleep(1)

    xclib.pxd_PIXCIclose()


if __name__ == '__main__':
    run_testcase()
