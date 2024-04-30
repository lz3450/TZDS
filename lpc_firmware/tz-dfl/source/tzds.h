#ifndef _TZDS_H_
#define _TZDS_H_

#define TZDS_RW __attribute__((annotate("tzds")))
#define TZDS_R __attribute__((annotate("tzds.read")))
#define TZDS_W __attribute__((annotate("tzds.write")))

#endif /* _TZDS_H_ */
