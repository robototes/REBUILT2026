package robotutils.perrobotconfig;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Collections;
import java.util.List;


/**
 * Utility to identify which RoboRIO is being used by checking its MAC address.
 */
public class MacAddress {

    /**
     * Checks if the current RoboRIO matches the provided ID bytes.
     *
     * @param identifier The last 3 hex pairs (e.g., 0x34, 0x4C, 0x2D).
     * @return true if the hardware matches.
     */
    public static boolean isRobot(int... identifier) {
        try {
            // Get all network ports (Ethernet, USB, etc.)
            List<NetworkInterface> interfaces = Collections.list(
                NetworkInterface.getNetworkInterfaces());

            for (NetworkInterface netIf : interfaces) {
                byte[] mac = netIf.getHardwareAddress();

                // Only check interfaces that actually have a MAC address
                if (mac != null && mac.length >= identifier.length) {
                    boolean match = true;

                    // Compare the end of the RIO's MAC to our ID bytes
                    for (int i = 0; i < identifier.length; i++) {
                        // Get the byte from the RIO and the byte from our ID
                        int macByte = mac[mac.length - identifier.length + i] & 0xFF;
                        int targetByte = identifier[i] & 0xFF;

                        // If any byte doesn't match, this isn't the right robot
                        if (macByte != targetByte) {
                            match = false;
                            break;
                        }
                    }
                    if (match) {
                        // Found a match!
                        return true;
                    }
                }
            }
        }
        catch (SocketException e) {
            System.err.println("[MacAddress] Could not read network ports: " + e.getMessage());
        }
        return false;
    }

    /**
     * Prints all MAC addresses to the console.
     * Use this in RobotInit to find the ID of a new robot.
     */
    public static void dumpToConsole() {
        try {
            Collections.list(NetworkInterface.getNetworkInterfaces()).forEach(netIf -> {
                try {
                    byte[] mac = netIf.getHardwareAddress();
                    if (mac != null) {
                        StringBuilder sb = new StringBuilder();
                        for (int i = 0; i < mac.length; i++) {
                            // Format bytes as uppercase hex (like 00:80:2F...)
                            sb.append(String.format(
                                "%02X%s", mac[i], (i < mac.length - 1) ? ":" : ""));
                        }
                        System.out.println(
                            "Interface: " + netIf.getName() + " | MAC: " + sb.toString());
                    }
                }
                catch (SocketException ignored) {
                    // Do nothing
                }
            });
        }
        catch (SocketException ignored) {
            // Do nothing
        }
    }
}
