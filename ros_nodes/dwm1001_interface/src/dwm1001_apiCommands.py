#!/usr/bin/env python
""" For more info on the documentation go to https://www.decawave.com/sites/default/files/dwm1001-api-guide.pdf
"""

class DWM1001_API_COMMANDS:
        DOUBLE_ENTER    = b'\r\r'   # ASCII char for double Enter
        SINGLE_ENTER    = b'\r'     # ASCII char for single Enter
        HELP            = b'?'      # Display help
        QUIT            = b'quit'   # Quit API shell mode
        GC              = b'gc'     # Clears GPIO pin
        GG              = b'gg'     # Reads GPIO pin level
        GS              = b'gs'     # Sets GPIO as output and sets its value
        GT              = b'gt'     # Toggle GPIO(must be an output)
        F               = b'f'      # Show free memory on the heap
        PS              = b'ps'     # Show info about running threads
        PMS             = b'pms'    # Show power managements tasks. IDL means that task is idle. USE means that task is allocated in the power management
        RESET           = b'reset'  # reset the dev board
        UT              = b'ut'     # Show device uptime
        FRST            = b'frst'   # Factory reset
        TWI             = b'twi'    # General purpose I2C/TWI read
        AID             = b'aid'    # Read ACC device ID
        AV              = b'av'     # Rad ACC values
        LES             = b'les'    # Show distances to ranging anchors and the position if location engine is enabled
        LEC             = b'lec'    # Show measurement and position in CSV format
        LEP             = b'lep'    # Show position in CSV format.Sending this command multiple times will turn on/off this functionality.
        SI              = b'si'     # System Info
        NMG             = b'nmg'    # Get node mode info
        NMO             = b'nmo'    # Enable passive offline option and resets the node
        NMP             = b'nmp'    # Enable active offline option and resets the node.
        NMA             = b'nma'    # Configures node to as anchor, active and reset the node.
        NMI             = b'nmi'    # Configures node to as anchor initiator, active and reset the node.
        NMT             = b'nmt'    # Configures node to as tag, active and reset the node
        NMTL            = b'nmtl'   # Configures node to as tag, active, low power and resets the node.
        BPC             = b'bpc'    # Toggle UWB bandwidth / tx power compensation.
        LA              = b'la'     # Show anchor list
        STG             = b'stg'    # Display statistics
        STC             = b'stc'    # Clears statistics
        TLV             = b'tlv'    # Parses given tlv frame, see section 4 for valid TLV commands
        AURS            = b'aurs'   # Set position update rate. See section 4.3.3 for more detail.
        AURG            = b'aurg'   # Get position update rate. See section 4.3.4 for more details
        APG             = b'apg'    # Get position of the node.See section 3.4.2 for more detail
        APS             = b'aps'    # Set position of the node.See section 3.4.2for more detail
        ACAS            = b'acas'   # Configures node as anchor with given options
        ACTS            = b'acts'   # Configures node as tag with given options
