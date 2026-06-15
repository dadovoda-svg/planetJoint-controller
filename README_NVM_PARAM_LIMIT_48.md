# NVM Parameter Table Capacity Increased to 48

This baseline increases the maximum number of runtime/NVM parameter items from 32 to 48.

## Changed value

```cpp
static constexpr uint8_t MAX_PARAMS = 48;
```

## Compatibility note

The persistent storage image contains a fixed-size array of `ParamItem items[MAX_PARAMS]`.
Therefore changing `MAX_PARAMS` changes `sizeof(StorageImage)`.

With the current storage implementation, an existing NVS blob saved with the previous 32-item layout will not be accepted by the firmware built with the 48-item layout. The NVS data is not physically erased by this change, but the firmware will reject the old image because its size does not match the new `StorageImage` size.

Recommended manual migration procedure:

1. Before flashing this firmware, run `export` from the USB console and save the parameter dump.
2. Flash the new firmware.
3. Re-import the saved parameters using `import`.
4. Run `save` to write the new 48-item storage image.

No automatic migration from 32 to 48 items is implemented in this baseline by design.
