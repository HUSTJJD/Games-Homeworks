#pragma once
#include <stddef.h>
#include <string.h>
#if defined(__cplusplus)
extern "C"
{
#endif
    const int no_argument = 0;
    const int required_argument = 1;
    const int optional_argument = 2;

    char *optarg;
    int optopt;
    int optind = 1;
    int opterr;

    static char *optcursor = NULL;

    struct option
    {
        const char *name;
        int has_arg;
        int *flag;
        int val;
    };

    int getopt(int argc, char *const argv[], const char *optstring)
    {
        int optchar = -1;
        const char *optdecl = NULL;
        optarg = NULL;
        opterr = 0;
        optopt = 0;
        if (optind >= argc)
            goto no_more_optchars;
        if (argv[optind] == NULL)
            goto no_more_optchars;
        if (*argv[optind] != '-')
            goto no_more_optchars;
        if (strcmp(argv[optind], "-") == 0)
            goto no_more_optchars;
        if (strcmp(argv[optind], "--") == 0)
        {
            ++optind;
            goto no_more_optchars;
        }
        if (optcursor == NULL || *optcursor == '\0')
            optcursor = argv[optind] + 1;
        optchar = *optcursor;
        optopt = optchar;
        optdecl = strchr(optstring, optchar);
        if (optdecl)
        {
            if (optdecl[1] == ':')
            {
                optarg = ++optcursor;
                if (*optarg == '\0')
                {
                    if (optdecl[2] != ':')
                    {
                        if (++optind < argc)
                        {
                            optarg = argv[optind];
                        }
                        else
                        {
                            optarg = NULL;
                            optchar = (optstring[0] == ':') ? ':' : '?';
                        }
                    }
                    else
                    {
                        optarg = NULL;
                    }
                }
                optcursor = NULL;
            }
        }
        else
        {
            optchar = '?';
        }
        if (optcursor == NULL || *++optcursor == '\0')
            ++optind;
        return optchar;
    no_more_optchars:
        optcursor = NULL;
        return -1;
    };

    int getopt_long(int argc, char *const argv[], const char *optstring,
                    const struct option *longopts, int *longindex)
    {
        const struct option *o = longopts;
        const struct option *match = NULL;
        int num_matches = 0;
        size_t argument_name_length = 0;
        const char *current_argument = NULL;
        int retval = -1;
        optarg = NULL;
        optopt = 0;
        if (optind >= argc)
            return -1;
        if (strlen(argv[optind]) < 3 || strncmp(argv[optind], "--", 2) != 0)
            return getopt(argc, argv, optstring);
        current_argument = argv[optind] + 2;
        argument_name_length = strcspn(current_argument, "=");
        for (; o->name; ++o)
        {
            if (strncmp(o->name, current_argument, argument_name_length) == 0)
            {
                match = o;
                ++num_matches;
            }
        }
        if (num_matches == 1)
        {
            if (longindex)
                *longindex = (int)(match - longopts);
            if (match->flag)
                *(match->flag) = match->val;
            retval = match->flag ? 0 : match->val;
            if (match->has_arg != no_argument)
            {
                optarg = strchr(argv[optind], '=');
                if (optarg != NULL)
                    ++optarg;
                if (match->has_arg == required_argument)
                {
                    if (optarg == NULL && ++optind < argc)
                    {
                        optarg = argv[optind];
                    }
                    if (optarg == NULL)
                        retval = ':';
                }
            }
            else if (strchr(argv[optind], '='))
            {
                retval = '?';
            }
        }
        else
        {
            retval = '?';
        }
        ++optind;
        return retval;
    }
#if defined(__cplusplus)
}
#endif