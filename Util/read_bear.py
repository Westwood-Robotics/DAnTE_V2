
from pybear import Manager

pbm = Manager.BEAR(port=UB000E, baudrate=8000000)

while True:
    try:
        status = pbm.get_bulk_status((1, 'present_position'),
                                     (2, 'present_position'),
                                     (3, 'present_position'))
        err = [data[1] for data in status]
        position = [data[0][0] for data in status]

        # Print positions
        # print("INDEX: %2.2f, INDEX_M: %2.2f, THUMB: %2.2f" % position)
        print(position)
        time.sleep(0.5)
    except KeyboardInterrupt:
        running = [0]
        print("User interrupted.")