package robotutils.dashboard;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import edu.wpi.first.networktables.StringPublisher;
import robotutils.pub.interfaces.dashboard.StringPublisherWrapper;

import org.junit.jupiter.api.Test;

class TestStringPublisherWrapper {

    @Test
    void firstNullValue_publishesEmptyString() {
        StringPublisher publisher = mock(StringPublisher.class);
        StringPublisherWrapper wrapper = new StringPublisherWrapper(publisher);

        wrapper.set(null);

        verify(publisher, times(1)).set("");
    }

    @Test
    void repeatedNullValue_publishesOnce() {
        StringPublisher publisher = mock(StringPublisher.class);
        StringPublisherWrapper wrapper = new StringPublisherWrapper(publisher);

        wrapper.set(null);
        wrapper.set(null);

        verify(publisher, times(1)).set("");
    }

    @Test
    void sameValueTwice_publishesOnce() {
        StringPublisher publisher = mock(StringPublisher.class);
        StringPublisherWrapper wrapper = new StringPublisherWrapper(publisher);

        wrapper.set("RobotUtils");
        wrapper.set("RobotUtils");

        verify(publisher, times(1)).set("RobotUtils");
    }

    @Test
    void changedValue_publishesAgain() {
        StringPublisher publisher = mock(StringPublisher.class);
        StringPublisherWrapper wrapper = new StringPublisherWrapper(publisher);

        wrapper.set("Comp");
        wrapper.set("Practice");

        verify(publisher, times(1)).set("Comp");
        verify(publisher, times(1)).set("Practice");
    }
}
