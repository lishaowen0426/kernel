hermit-root := `pwd`


default:
    just --list

loader:
    cd {{hermit-root}}/loader && cargo xtask build --target aarch64 

qemu:
   qemu-system-aarch64 \
        -machine virt,gic-version=3,mte=on \
        -cpu cortex-a710 \
        -qmp tcp:localhost:14576,server,wait=off \
        -smp 1 \
        -device virtio-blk-pci,drive=hd,disable-legacy=on -drive file=disk.img,if=none,id=hd \
        -m 512M  \
        -semihosting \
        -display none -serial stdio \
        -kernel {{hermit-root}}/loader/target/aarch64/debug/hermit-loader \
        -device guest-loader,addr=0x48000000,initrd={{hermit-root}}/hermit-rs-template/target/aarch64-unknown-hermit/debug/hermit-rs-template  

fuse:
    #!/usr/bin/env sh
    sudo -E env "PATH=$PATH" virtiofsd --socket-path=/tmp/vhostqemu -o source=/home/sw/hermit/testdir -o cache=always &
    sudo qemu-system-aarch64 \
        -machine virt,gic-version=3,mte=on \
        -cpu cortex-a710 \
        -qmp tcp:localhost:14576,server,wait=off \
        -chardev socket,id=char0,path=/tmp/vhostqemu \
        -device vhost-user-fs-pci,queue-size=1024,chardev=char0,tag=myfs \
        -smp 1 \
        -device virtio-blk-pci,drive=hd,disable-legacy=on -drive file=disk.img,if=none,id=hd \
        -m 1G -object memory-backend-file,id=mem,size=1G,mem-path=/dev/shm,share=on -numa node,memdev=mem  \
        -semihosting \
        -display none -serial stdio \
        -kernel {{hermit-root}}/loader/target/aarch64/debug/hermit-loader \
        -device guest-loader,addr=0x48000000,initrd={{hermit-root}}/hermit-rs-template/target/aarch64-unknown-hermit/debug/hermit-rs-template  

qemu-dtb:
   qemu-system-aarch64 \
        -machine virt,gic-version=3,mte=on,dumpdtb=dump.dtb \
        -cpu cortex-a710 \
        -qmp tcp:localhost:14576,server,wait=off \
        -smp 1 \
        -device virtio-blk-pci,drive=hd -drive file=disk.img,if=none,id=hd \
        -m 512M  \
        -semihosting \
        -display none -serial stdio \
        -kernel {{hermit-root}}/loader/target/aarch64/debug/hermit-loader \
        -device guest-loader,addr=0x48000000,initrd={{hermit-root}}/hermit-rs-template/target/aarch64-unknown-hermit/debug/hermit-rs-template -s -S

#build demo app
demo:
    cd {{hermit-root}}/hermit-rs-template && cargo clean     &&cargo build    --features "syscall,pci,fs"   --target aarch64-unknown-hermit 




    
lldb:
    lldb --source lldb_cmd
    
