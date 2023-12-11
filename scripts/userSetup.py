# # -*- coding: utf-8 -*-
import sys
import textwrap

from maya import (
    cmds,
    mel,
)


if sys.version_info > (3, 0):
    from typing import TYPE_CHECKING
    if TYPE_CHECKING:
        from typing import (
            Optional,  # noqa: F401
            Dict,  # noqa: F401
            List,  # noqa: F401
            Tuple,  # noqa: F401
            Pattern,  # noqa: F401
            Callable,  # noqa: F401
            Any,  # noqa: F401
            Text,  # noqa: F401
            Generator,  # noqa: F401
            Union,  # noqa: F401
            Iterable # noqa: F401
        )


##############################################################################
def register_menu():
    # type: () -> None
    """Setup menu"""

    if cmds.about(batch=True):
        return

    if not is_default_window_menu_registered():
        register_default_window_menu()

    cmds.setParent("MayaWindow|mainWindowMenu", menu=True)

    cmds.menuItem(divider=True)
    item = cmds.menuItem(
        "add_intersection_marker",
        label="Intersection Marker",
        annotation="Show intersection marker",
        echoCommand=True,
        command=textwrap.dedent(
            """
                cmds.loadPlugin("MayaIntersectionMarker.mll", quiet=True)
                cmds.intersectionMarker()
            """)
    )
    print("IntersectionMarker: register menu item as {}".format(item))


def is_default_window_menu_registered():
    # type: () -> bool
    """Check if default Window menu is registered"""
    if not cmds.menu("MayaWindow|mainWindowMenu", exists=True):
        return False

    kids = cmds.menu("MayaWindow|mainWindowMenu", query=True, itemArray=True)
    if not kids:
        return False

    if len(kids) == 0:
        return False

    return True


def register_default_window_menu():
    cmd = '''
    buildViewMenu MayaWindow|mainWindowMenu;
    setParent -menu "MayaWindow|mainWindowMenu";
    '''

    mel.eval(cmd)


def deregister_menu():
    # type: () -> None
    """Remove menu"""

    if cmds.about(batch=True):
        return

    try:
        path = "MayaWindow|mainWindowMenu|add_intersection_marker"
        cmds.deleteUI(path, menuItem=True)

    except Exception as e:
        import traceback
        traceback.print_exc()
        print(e)


def __register_menu():
    """Setup menu"""

    from textwrap import dedent
    cmds.evalDeferred(dedent(
        """
        try:
            register_menu()
        except:
            import traceback
            traceback.print_exc()
        """
    ))


if __name__ == '__main__':
    try:
        __register_menu()

    except Exception:
        # avoidng the "call userSetup.py chain" accidentally stop,
        # all exception must collapse
        import traceback
        traceback.print_exc()
