target extended-remote localhost:3333
set remotetimeout 30
set print pretty on

define flashwrite
  monitor flash write_image erase $arg0
end

define nuttx_update
  reset_halt
  flashwrite nuttx
end

define reset_halt
  monitor reset halt
end

define reset_run
  monitor reset halt
  continue
end

define hookpost-symbol-file
  eval "monitor nuttx.pid_offset %d", &((struct tcb_s *)(0))->pid
  eval "monitor nuttx.xcpreg_offset %d", &((struct tcb_s *)(0))->xcp.regs
  eval "monitor nuttx.state_offset %d", &((struct tcb_s *)(0))->task_state
  eval "monitor nuttx.name_offset %d", &((struct tcb_s *)(0))->name
  eval "monitor nuttx.name_size %d", sizeof(((struct tcb_s *)(0))->name)
end
