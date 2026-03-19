package frc.robot.utils;

import java.util.ArrayList;

public final class ObjectPool<T> {

    private final ArrayList<T> items;
    private final T[] emptyArray;

    public ObjectPool(T[] emptyArray, int initialCapacity) {
        this.emptyArray = emptyArray;
        this.items = new ArrayList<>(initialCapacity);
    }

    public void clear() {
        items.clear();
    }

    public ArrayList<T> list() {
        return items;
    }

    public T[] toArray() {
        return items.isEmpty() ? emptyArray : items.toArray(emptyArray);
    }
}
