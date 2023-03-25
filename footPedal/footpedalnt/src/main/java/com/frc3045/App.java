package com.frc3045;

/**
 * Hello world!
 */
public final class App {
    private App() {
    }

    /**
     * Says hello to the world.
     * @param args The arguments of the program.
     */
    public static void main(String[] args) {
        System.out.println("Hello World!");
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("datatable");

        // get a topic from a NetworkTableInstance
        // the topic name in this case is the full name
        DoubleTopic dblTopic = inst.getDoubleTopic("/datatable/X");

        // get a topic from a NetworkTable
        // the topic name in this case is the name within the table;
        // this line and the one above reference the same topic
        DoubleTopic dblTopic = table.getDoubleTopic("X");

        // get a type-specific topic from a generic Topic
        Topic genericTopic = inst.getTopic("/datatable/X");
        DoubleTopic dblTopic = new DoubleTopic(genericTopic);
    }
}
